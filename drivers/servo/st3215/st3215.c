#include <zephyr/device.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <stdint.h>
#include <string.h>

#define DT_DRV_COMPAT waveshare_st3215

#define ST3215_HEADER         0xFF
#define ST3215_BROADCAST_ID   0xFE

#define ST3215_INST_PING      0x01
#define ST3215_INST_READ      0x02
#define ST3215_INST_WRITE     0x03
#define ST3215_INST_REGWRITE  0x04
#define ST3215_INST_ACTION    0x05
#define ST3215_INST_SYNCWRITE 0x83
#define ST3215_INST_RESET     0x06

#define ST3215_REG_TARGET_POSITION   0x2A
#define ST3215_REG_GOAL_TIME         0x2C
#define ST3215_REG_GOAL_SPEED        0x2E
#define ST3215_REG_TORQUE_ENABLE     0x28
#define ST3215_REG_CURRENT_POSITION  0x38
#define ST3215_REG_STATUS_RETURN     0x08
#define ST3215_REG_OPERATING_MODE    0x21
#define ST3215_REG_SERVO_STATUS      0x41
#define ST3215_REG_PRESENT_SPEED     0x3A
#define ST3215_REG_PRESENT_LOAD      0x3C
#define ST3215_REG_PRESENT_VOLTAGE   0x3E
#define ST3215_REG_PRESENT_TEMP      0x3F
#define ST3215_REG_MOVING            0x42
#define ST3215_REG_LOCK              0x37

#define ST3215_POSITION_MAX   4095
#define ST3215_MAX_PACKET_LEN 16
#define ST3215_RX_BUF_SIZE    32

LOG_MODULE_REGISTER(st3215_servo, LOG_LEVEL_DBG);

struct st3215_bus {
    const struct device *uart_dev;
    struct k_mutex lock;
    struct k_sem tx_done;
    struct k_sem rx_done;
    uint8_t rx_buf[ST3215_RX_BUF_SIZE];
    size_t rx_len;
    int tx_status;
    int rx_status;
    bool callback_set;
};

struct st3215_config {
    struct st3215_bus *bus;
    uint8_t servo_id;
    uint16_t speed;
    uint16_t time_ms;
};

struct st3215_data {
    uint16_t speed;
    uint16_t time_ms;
};

static uint8_t st3215_compute_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return (uint8_t)(~checksum);
}

static void st3215_uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    struct st3215_bus *bus = user_data;

    switch (evt->type) {
    case UART_TX_DONE:
        bus->tx_status = 0;
        k_sem_give(&bus->tx_done);
        break;

    case UART_TX_ABORTED:
        bus->tx_status = -EIO;
        k_sem_give(&bus->tx_done);
        break;

    case UART_RX_RDY:
        bus->rx_len = evt->data.rx.len;
        bus->rx_status = 0;
        k_sem_give(&bus->rx_done);
        break;

    case UART_RX_STOPPED:
        bus->rx_len = evt->data.rx_stop.data.len;
        bus->rx_status = 0;
        k_sem_give(&bus->rx_done);
        break;

    case UART_RX_BUF_REQUEST:
    case UART_RX_BUF_RELEASED:
    case UART_RX_DISABLED:
        break;
    }
}

static int st3215_bus_tx(struct st3215_bus *bus, const uint8_t *data, size_t len)
{
    int ret;

    k_sem_reset(&bus->tx_done);
    bus->tx_status = -EAGAIN;

    ret = uart_tx(bus->uart_dev, data, len, SYS_FOREVER_US);
    if (ret < 0) {
        LOG_ERR("uart_tx failed: %d", ret);
        return ret;
    }

    ret = k_sem_take(&bus->tx_done, K_MSEC(100));
    if (ret < 0) {
        LOG_ERR("TX timeout");
        uart_tx_abort(bus->uart_dev);
        return -ETIMEDOUT;
    }

    if (bus->tx_status < 0) {
        LOG_ERR("TX failed: %d", bus->tx_status);
        return bus->tx_status;
    }

    return 0;
}

static int st3215_bus_rx(struct st3215_bus *bus, uint32_t timeout_ms)
{
    int ret;

    k_sem_reset(&bus->rx_done);
    bus->rx_len = 0;
    bus->rx_status = -EAGAIN;

    ret = uart_rx_enable(bus->uart_dev, bus->rx_buf, sizeof(bus->rx_buf),
                         timeout_ms * 1000);
    if (ret < 0) {
        LOG_ERR("uart_rx_enable failed: %d", ret);
        return ret;
    }

    ret = k_sem_take(&bus->rx_done, K_MSEC(timeout_ms + 50));
    if (ret < 0) {
        LOG_DBG("RX timeout (no response or broadcast)");
        uart_rx_disable(bus->uart_dev);
        return -ETIMEDOUT;
    }

    if (bus->rx_status < 0) {
        LOG_ERR("RX failed: %d", bus->rx_status);
        uart_rx_disable(bus->uart_dev);
        return bus->rx_status;
    }

    if (bus->rx_len < 6) {
        LOG_ERR("RX too short: %zu bytes", bus->rx_len);
        uart_rx_disable(bus->uart_dev);
        return -EIO;
    }

    uint8_t computed_cs = st3215_compute_checksum(bus->rx_buf + 2, bus->rx_len - 3);
    if (bus->rx_buf[bus->rx_len - 1] != computed_cs) {
        LOG_ERR("Checksum mismatch: expected 0x%02X, got 0x%02X",
                computed_cs, bus->rx_buf[bus->rx_len - 1]);
        uart_rx_disable(bus->uart_dev);
        return -EIO;
    }

    uart_rx_disable(bus->uart_dev);

    return bus->rx_len;
}

static int st3215_send_packet(struct st3215_bus *bus,
                              uint8_t servo_id,
                              uint8_t instruction,
                              const uint8_t *params,
                              size_t param_len)
{
    size_t packet_len = 6 + param_len;
    uint8_t packet[ST3215_MAX_PACKET_LEN];

    if (param_len > 10) {
        return -EINVAL;
    }

    packet[0] = ST3215_HEADER;
    packet[1] = ST3215_HEADER;
    packet[2] = servo_id;
    packet[3] = param_len + 2;
    packet[4] = instruction;

    for (size_t i = 0; i < param_len; i++) {
        packet[5 + i] = params[i];
    }

    packet[5 + param_len] = st3215_compute_checksum(packet + 2, 3 + param_len);

    LOG_DBG("TX: %02X %02X %02X %02X %02X", packet[0], packet[1], packet[2], packet[3], packet[4]);
    if (param_len > 0) {
        LOG_HEXDUMP_DBG(params, param_len, "TX params:");
    }

    return st3215_bus_tx(bus, packet, packet_len);
}

static int st3215_read_response(struct st3215_bus *bus, uint32_t timeout_ms)
{
    return st3215_bus_rx(bus, timeout_ms);
}

static int st3215_enable_torque(struct st3215_bus *bus, uint8_t servo_id)
{
    uint8_t params[2] = {
        ST3215_REG_TORQUE_ENABLE,
        1
    };

    return st3215_send_packet(bus, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_disable_torque(struct st3215_bus *bus, uint8_t servo_id)
{
    uint8_t params[2] = {
        ST3215_REG_TORQUE_ENABLE,
        0
    };

    return st3215_send_packet(bus, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_set_status_return_level(struct st3215_bus *bus, uint8_t servo_id)
{
    uint8_t params[2] = {
        ST3215_REG_STATUS_RETURN,
        1
    };

    return st3215_send_packet(bus, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_set_operating_mode(struct st3215_bus *bus, uint8_t servo_id)
{
    uint8_t params[2] = {
        ST3215_REG_OPERATING_MODE,
        0
    };

    return st3215_send_packet(bus, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_unlock_eeprom(struct st3215_bus *bus, uint8_t servo_id)
{
    uint8_t params[2] = {
        ST3215_REG_LOCK,
        0
    };

    return st3215_send_packet(bus, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_lock_eeprom(struct st3215_bus *bus, uint8_t servo_id)
{
    uint8_t params[2] = {
        ST3215_REG_LOCK,
        1
    };

    return st3215_send_packet(bus, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_set_position(const struct device *dev, int32_t angle_mdeg)
{
    const struct st3215_config *config = dev->config;
    struct st3215_data *data = dev->data;
    struct st3215_bus *bus = config->bus;

    if (angle_mdeg < 0) {
        angle_mdeg = 0;
    }
    if (angle_mdeg > 360000) {
        angle_mdeg = 360000;
    }

    uint16_t raw_position = (uint16_t)((angle_mdeg * ST3215_POSITION_MAX) / 360000);

    uint8_t params[7];
    params[0] = ST3215_REG_TARGET_POSITION;
    params[1] = (uint8_t)(raw_position & 0xFF);
    params[2] = (uint8_t)((raw_position >> 8) & 0xFF);
    params[3] = (uint8_t)(data->time_ms & 0xFF);
    params[4] = (uint8_t)((data->time_ms >> 8) & 0xFF);
    params[5] = (uint8_t)(data->speed & 0xFF);
    params[6] = (uint8_t)((data->speed >> 8) & 0xFF);

    k_mutex_lock(&bus->lock, K_FOREVER);
    int ret = st3215_send_packet(bus, config->servo_id, ST3215_INST_WRITE, params, 7);
    k_mutex_unlock(&bus->lock);

    return ret;
}

static int st3215_get_position(const struct device *dev, int32_t *angle_mdeg)
{
    const struct st3215_config *config = dev->config;
    struct st3215_bus *bus = config->bus;
    uint8_t read_params[2] = {
        ST3215_REG_CURRENT_POSITION,
        2
    };

    k_mutex_lock(&bus->lock, K_FOREVER);

    int ret = st3215_send_packet(bus, config->servo_id, ST3215_INST_READ, read_params, 2);
    if (ret < 0) {
        k_mutex_unlock(&bus->lock);
        return ret;
    }

    ret = st3215_read_response(bus, 50);
    k_mutex_unlock(&bus->lock);

    if (ret < 0) {
        LOG_ERR("Failed to read position response");
        return ret;
    }

    uint16_t raw_position = bus->rx_buf[5] | (bus->rx_buf[6] << 8);
    *angle_mdeg = (int32_t)((raw_position * 360000) / ST3215_POSITION_MAX);

    LOG_DBG("Raw position: %u, angle: %d.%03u deg",
            raw_position, (*angle_mdeg / 1000), (*angle_mdeg % 1000));

    return 0;
}

static int st3215_set_speed(const struct device *dev, uint16_t speed)
{
    struct st3215_data *data = dev->data;
    data->speed = speed;
    LOG_DBG("Speed set to %u", speed);
    return 0;
}

static int st3215_set_time(const struct device *dev, uint16_t time_ms)
{
    struct st3215_data *data = dev->data;
    data->time_ms = time_ms;
    LOG_DBG("Time set to %u ms", time_ms);
    return 0;
}

static int st3215_ping(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    struct st3215_bus *bus = config->bus;

    k_mutex_lock(&bus->lock, K_FOREVER);

    int ret = st3215_send_packet(bus, config->servo_id, ST3215_INST_PING, NULL, 0);
    if (ret < 0) {
        k_mutex_unlock(&bus->lock);
        return ret;
    }

    ret = st3215_read_response(bus, 100);
    k_mutex_unlock(&bus->lock);

    if (ret < 0) {
        LOG_ERR("Ping failed: no response");
        return ret;
    }

    uint8_t status = bus->rx_buf[4];
    LOG_INF("Ping response: servo status = 0x%02X", status);

    return (status == 0) ? 0 : -EIO;
}

static int st3215_enable_torque_api(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    struct st3215_bus *bus = config->bus;

    k_mutex_lock(&bus->lock, K_FOREVER);
    int ret = st3215_enable_torque(bus, config->servo_id);
    k_mutex_unlock(&bus->lock);

    return ret;
}

static int st3215_disable_torque_api(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    struct st3215_bus *bus = config->bus;

    k_mutex_lock(&bus->lock, K_FOREVER);
    int ret = st3215_disable_torque(bus, config->servo_id);
    k_mutex_unlock(&bus->lock);

    return ret;
}

static int st3215_get_status(const struct device *dev, uint8_t *status)
{
    const struct st3215_config *config = dev->config;
    struct st3215_bus *bus = config->bus;
    uint8_t read_params[2] = {
        ST3215_REG_SERVO_STATUS,
        1
    };

    k_mutex_lock(&bus->lock, K_FOREVER);

    int ret = st3215_send_packet(bus, config->servo_id, ST3215_INST_READ, read_params, 2);
    if (ret < 0) {
        k_mutex_unlock(&bus->lock);
        return ret;
    }

    ret = st3215_read_response(bus, 100);
    k_mutex_unlock(&bus->lock);

    if (ret < 0) {
        LOG_ERR("Failed to read status response (ID=%u)", config->servo_id);
        return ret;
    }

    *status = bus->rx_buf[5];

    if (*status & BIT(0)) LOG_WRN("Servo %u: Voltage error", config->servo_id);
    if (*status & BIT(1)) LOG_WRN("Servo %u: Temperature error", config->servo_id);
    if (*status & BIT(2)) LOG_WRN("Servo %u: Overload error", config->servo_id);
    if (*status & BIT(3)) LOG_WRN("Servo %u: Encoder error", config->servo_id);
    if (*status & BIT(4)) LOG_WRN("Servo %u: Overcurrent error", config->servo_id);
    if (*status & BIT(5)) LOG_WRN("Servo %u: Angle limit exceeded", config->servo_id);

    LOG_DBG("Servo %u status: 0x%02X", config->servo_id, *status);

    return 0;
}

static int st3215_init(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    struct st3215_data *data = dev->data;
    struct st3215_bus *bus = config->bus;

    if (!device_is_ready(bus->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    data->speed = config->speed;
    data->time_ms = config->time_ms;

    if (!bus->callback_set) {
        k_mutex_lock(&bus->lock, K_FOREVER);

        if (!bus->callback_set) {
            int ret = uart_callback_set(bus->uart_dev, st3215_uart_callback, bus);
            if (ret < 0) {
                LOG_ERR("Failed to set UART callback: %d", ret);
                k_mutex_unlock(&bus->lock);
                return ret;
            }
            bus->callback_set = true;
        }

        k_mutex_unlock(&bus->lock);
    }

    k_usleep(5000);

    k_mutex_lock(&bus->lock, K_FOREVER);

    int ret;

    // ret = st3215_unlock_eeprom(bus, config->servo_id);
    // if (ret < 0) {
    //     LOG_ERR("Failed to unlock EEPROM (ID=%u): %d", config->servo_id, ret);
    // }
    // k_usleep(1000);

    ret = st3215_set_status_return_level(bus, config->servo_id);
    if (ret < 0) {
        LOG_ERR("Failed to set status return level (ID=%u): %d", config->servo_id, ret);
    }
    k_usleep(1000);

    ret = st3215_set_operating_mode(bus, config->servo_id);
    if (ret < 0) {
        LOG_ERR("Failed to set operating mode to servo mode (ID=%u): %d", config->servo_id, ret);
    }
    k_usleep(1000);

    ret = st3215_enable_torque(bus, config->servo_id);
    if (ret < 0) {
        LOG_ERR("Failed to enable torque (ID=%u): %d", config->servo_id, ret);
    }

    k_mutex_unlock(&bus->lock);

    LOG_INF("ST3215 servo driver initialized (ID=%u, speed=%u, time=%ums)",
            config->servo_id, data->speed, data->time_ms);

    return 0;
}

static const struct servo_driver_api st3215_api = {
    .set_position = st3215_set_position,
    .get_position = st3215_get_position,
    .set_speed = st3215_set_speed,
    .set_time = st3215_set_time,
    .ping = st3215_ping,
    .enable_torque = st3215_enable_torque_api,
    .disable_torque = st3215_disable_torque_api,
    .get_status = st3215_get_status,
};

#define ST3215_DEFINE(n)                                                       \
    static struct st3215_bus st3215_bus_##n = {                                \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                             \
    };                                                                         \
    static struct st3215_data st3215_data_##n;                                 \
    static const struct st3215_config st3215_config_##n = {                    \
        .bus = &st3215_bus_##n,                                                \
        .servo_id = DT_INST_PROP(n, servo_id),                                 \
        .speed = DT_INST_PROP_OR(n, speed, 0),                                 \
        .time_ms = DT_INST_PROP_OR(n, time_ms, 0),                             \
    };                                                                         \
    static int st3215_init_##n(const struct device *dev)                       \
    {                                                                          \
        struct st3215_bus *bus = &st3215_bus_##n;                              \
        if (!bus->callback_set) {                                              \
            k_mutex_init(&bus->lock);                                          \
            k_sem_init(&bus->tx_done, 0, K_SEM_MAX_LIMIT);                     \
            k_sem_init(&bus->rx_done, 0, K_SEM_MAX_LIMIT);                     \
        }                                                                      \
        return st3215_init(dev);                                               \
    }                                                                          \
    DEVICE_DT_INST_DEFINE(n,                                                   \
                          st3215_init_##n,                                     \
                          NULL,                                                \
                          &st3215_data_##n,                                    \
                          &st3215_config_##n,                                  \
                          POST_KERNEL,                                         \
                          CONFIG_SERVO_INIT_PRIORITY,                          \
                          &st3215_api);

DT_INST_FOREACH_STATUS_OKAY(ST3215_DEFINE)
