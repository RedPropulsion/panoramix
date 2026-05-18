#include <zephyr/device.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
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

LOG_MODULE_REGISTER(st3215_servo, LOG_LEVEL_DBG);

struct st3215_config {
    const struct device *uart_dev;
    const struct device *dir_gpio;
    uint8_t servo_id;
    uint16_t speed;
    uint16_t time_ms;
};

struct st3215_data {
    uint16_t speed;
    uint16_t time_ms;
    uint8_t rx_buf[32];
};

static uint8_t st3215_compute_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return (uint8_t)(~checksum);
}

static void st3215_set_tx_mode(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    if (config->dir_gpio != NULL) {
        gpio_pin_set(config->dir_gpio, 0, 1);
    }
}

static void st3215_set_rx_mode(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    if (config->dir_gpio != NULL) {
        gpio_pin_set(config->dir_gpio, 0, 0);
    }
}

static int st3215_send_packet(const struct device *dev,
                              uint8_t servo_id,
                              uint8_t instruction,
                              const uint8_t *params,
                              size_t param_len)
{
    const struct st3215_config *config = dev->config;

    size_t packet_len = 6 + param_len;
    uint8_t packet[16];

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

    st3215_set_tx_mode(dev);
    k_usleep(10);

    LOG_DBG("TX: %02X %02X %02X %02X %02X", packet[0], packet[1], packet[2], packet[3], packet[4]);
    if (param_len > 0) {
        LOG_HEXDUMP_DBG(params, param_len, "TX params:");
    }

    for (size_t i = 0; i < packet_len; i++) {
        uart_poll_out(config->uart_dev, packet[i]);
    }

    k_usleep(100);

    st3215_set_rx_mode(dev);
    k_usleep(50);

    return 0;
}

static int st3215_read_response(const struct device *dev,
                                uint8_t *buffer,
                                size_t expected_len,
                                uint32_t timeout_ms)
{
    const struct st3215_config *config = dev->config;
    size_t received = 0;
    uint32_t start_time = k_uptime_get_32();
    bool found_header = false;

    while ((k_uptime_get_32() - start_time) < timeout_ms) {
        uint8_t c;
        if (uart_poll_in(config->uart_dev, &c) == 0) {
            if (!found_header) {
                if (c == ST3215_HEADER) {
                    if (received == 0) {
                        received = 1;
                        buffer[0] = c;
                    } else if (received == 1 && buffer[0] == ST3215_HEADER) {
                        buffer[1] = c;
                        found_header = true;
                        received = 2;
                    } else {
                        buffer[0] = c;
                        received = 1;
                    }
                } else {
                    received = 0;
                }
            } else {
                buffer[received++] = c;
                if (received >= expected_len) {
                    break;
                }
            }
        } else {
            k_usleep(100);
        }
    }

    if (!found_header || received < 6) {
        return -ETIMEDOUT;
    }

    uint8_t computed_cs = st3215_compute_checksum(buffer + 2, received - 3);
    if (buffer[received - 1] != computed_cs) {
        LOG_ERR("Checksum mismatch: expected 0x%02X, got 0x%02X",
                computed_cs, buffer[received - 1]);
        return -EIO;
    }

    return received;
}

static int st3215_enable_torque(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    uint8_t params[2] = {
        ST3215_REG_TORQUE_ENABLE,
        1
    };

    return st3215_send_packet(dev, config->servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_disable_torque(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    uint8_t params[2] = {
        ST3215_REG_TORQUE_ENABLE,
        0
    };

    return st3215_send_packet(dev, config->servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_set_status_return_level(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    uint8_t params[2] = {
        ST3215_REG_STATUS_RETURN,
        1
    };

    return st3215_send_packet(dev, config->servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_set_operating_mode(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    uint8_t params[2] = {
        ST3215_REG_OPERATING_MODE,
        0
    };

    return st3215_send_packet(dev, config->servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_unlock_eeprom(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    uint8_t params[2] = {
        ST3215_REG_LOCK,
        0
    };

    return st3215_send_packet(dev, config->servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_lock_eeprom(const struct device *dev)
{
    const struct st3215_config *config = dev->config;
    uint8_t params[2] = {
        ST3215_REG_LOCK,
        1
    };

    return st3215_send_packet(dev, config->servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_set_position(const struct device *dev, int32_t angle_mdeg)
{
    const struct st3215_config *config = dev->config;
    struct st3215_data *data = dev->data;

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

    return st3215_send_packet(dev, config->servo_id, ST3215_INST_WRITE, params, 7);
}

static int st3215_get_position(const struct device *dev, int32_t *angle_mdeg)
{
    const struct st3215_config *config = dev->config;
    uint8_t response[16];
    uint8_t read_params[2] = {
        ST3215_REG_CURRENT_POSITION,
        2
    };

    int ret = st3215_send_packet(dev, config->servo_id, ST3215_INST_READ, read_params, 2);
    if (ret < 0) {
        return ret;
    }

    k_usleep(1000);

    ret = st3215_read_response(dev, response, 8, 50);
    if (ret < 0) {
        LOG_ERR("Failed to read position response");
        return ret;
    }

    uint16_t raw_position = response[5] | (response[6] << 8);
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
    uint8_t response[16];

    int ret = st3215_send_packet(dev, config->servo_id, ST3215_INST_PING, NULL, 0);
    if (ret < 0) {
        return ret;
    }

    k_usleep(1000);

    ret = st3215_read_response(dev, response, 6, 100);
    if (ret < 0) {
        LOG_ERR("Ping failed: no response");
        return ret;
    }

    uint8_t status = response[4];
    LOG_INF("Ping response: servo status = 0x%02X", status);

    return (status == 0) ? 0 : -EIO;
}

static int st3215_enable_torque_api(const struct device *dev)
{
    return st3215_enable_torque(dev);
}

static int st3215_disable_torque_api(const struct device *dev)
{
    return st3215_disable_torque(dev);
}

static int st3215_get_status(const struct device *dev, uint8_t *status)
{
    const struct st3215_config *config = dev->config;
    uint8_t response[16];
    uint8_t read_params[2] = {
        ST3215_REG_SERVO_STATUS,
        1
    };

    int ret = st3215_send_packet(dev, config->servo_id, ST3215_INST_READ, read_params, 2);
    if (ret < 0) {
        return ret;
    }

    k_usleep(1000);

    ret = st3215_read_response(dev, response, 7, 100);
    if (ret < 0) {
        LOG_ERR("Failed to read status response (ID=%u)", config->servo_id);
        return ret;
    }

    *status = response[5];

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

    if (!device_is_ready(config->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    data->speed = config->speed;
    data->time_ms = config->time_ms;

    k_usleep(5000);

    int ret;

    ret = st3215_unlock_eeprom(dev);
    if (ret < 0) {
        LOG_ERR("Failed to unlock EEPROM (ID=%u): %d", config->servo_id, ret);
    }
    k_usleep(1000);

    ret = st3215_set_status_return_level(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set status return level (ID=%u): %d", config->servo_id, ret);
    }
    k_usleep(1000);

    ret = st3215_set_operating_mode(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set operating mode to servo mode (ID=%u): %d", config->servo_id, ret);
    }
    k_usleep(1000);

    ret = st3215_lock_eeprom(dev);
    if (ret < 0) {
        LOG_ERR("Failed to lock EEPROM (ID=%u): %d", config->servo_id, ret);
    }
    k_usleep(1000);

    ret = st3215_enable_torque(dev);
    if (ret < 0) {
        LOG_ERR("Failed to enable torque (ID=%u): %d", config->servo_id, ret);
    }

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

#define ST3215_DEFINE(inst)                                                  \
    static struct st3215_data st3215_data_##inst;                            \
    static const struct st3215_config st3215_config_##inst = {               \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                        \
        .servo_id = DT_INST_PROP(inst, servo_id),                            \
        .speed = DT_INST_PROP_OR(inst, speed, 0),                            \
        .time_ms = DT_INST_PROP_OR(inst, time_ms, 0),                        \
    };                                                                       \
    DEVICE_DT_INST_DEFINE(inst,                                              \
                          st3215_init,                                       \
                          NULL,                                              \
                          &st3215_data_##inst,                               \
                          &st3215_config_##inst,                             \
                          POST_KERNEL,                                       \
                          CONFIG_SERVO_INIT_PRIORITY,                        \
                          &st3215_api);

DT_INST_FOREACH_STATUS_OKAY(ST3215_DEFINE)