#include <stdbool.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdint.h>
#include <string.h>

#define DT_DRV_COMPAT waveshare_st3215

#define ST3215_HEADER 0xFF
#define ST3215_BROADCAST_ID 0xFE

#define ST3215_INST_PING 0x01
#define ST3215_INST_READ 0x02
#define ST3215_INST_WRITE 0x03
#define ST3215_INST_REGWRITE 0x04
#define ST3215_INST_ACTION 0x05
#define ST3215_INST_SYNCWRITE 0x83
#define ST3215_INST_RESET 0x06

#define ST3215_REG_TARGET_POSITION 0x2A
#define ST3215_REG_GOAL_TIME 0x2C
#define ST3215_REG_GOAL_SPEED 0x2E
#define ST3215_REG_TORQUE_ENABLE 0x28
#define ST3215_REG_CURRENT_POSITION 0x38
#define ST3215_REG_STATUS_RETURN 0x08
#define ST3215_REG_OPERATING_MODE 0x21
#define ST3215_REG_SERVO_STATUS 0x41
#define ST3215_REG_PRESENT_SPEED 0x3A
#define ST3215_REG_PRESENT_LOAD 0x3C
#define ST3215_REG_PRESENT_VOLTAGE 0x3E
#define ST3215_REG_PRESENT_TEMP 0x3F
#define ST3215_REG_MOVING 0x42
#define ST3215_REG_LOCK 0x37

#define ST3215_POSITION_MAX 4095
#define ST3215_MAX_PACKET_LEN 16
#define ST3215_RX_BUF_SIZE 32

LOG_MODULE_REGISTER(st3215_servo, LOG_LEVEL_DBG);

#define RX_DONE BIT(0)
#define TX_DONE BIT(1)

K_EVENT_DEFINE(uart_servo_bus_event);

struct st3215_bus {
  const struct device *uart_dev;
  struct k_mutex lock;
  struct k_sem tx_done;
  struct k_sem rx_done;
  uint8_t rx_buf[ST3215_RX_BUF_SIZE];
  uint8_t rx_buf2[ST3215_RX_BUF_SIZE];
  uint8_t tx_buf[ST3215_MAX_PACKET_LEN];
  uint8_t *cur_rx_buf;
  uint8_t *next_rx_buf;
  size_t rx_len;
  size_t rx_offset;
  int tx_status;
  int rx_status;
  bool callback_set;
};

static struct st3215_bus __bus_state;

struct st3215_config {
  uint8_t servo_id;
  uint16_t speed;
  uint16_t time_ms;
  const struct device *uart_dev;
};

struct st3215_data {
  uint16_t speed;
  uint16_t time_ms;
  uint8_t rx_buff[ST3215_RX_BUF_SIZE];
  size_t rx_len;
  // size_t rx_offset;
};

static uint8_t st3215_compute_checksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum += data[i];
  }
  return (uint8_t)(~checksum);
}

static void st3215_uart_callback(const struct device *dev,
                                 struct uart_event *evt, void *user_data) {
  struct st3215_bus *bus = &__bus_state;
  int err;

  switch (evt->type) {
  case UART_TX_DONE:
    bus->tx_status = 0;
    LOG_WRN("TX DONE");
    k_event_post(&uart_servo_bus_event, TX_DONE);
    break;

  case UART_TX_ABORTED:
    LOG_WRN("TX ABORTED");
    bus->tx_status = -EIO;
    k_event_post(&uart_servo_bus_event, TX_DONE);
    break;

  case UART_RX_RDY:
    LOG_WRN("RX DONE");
    bus->rx_len = evt->data.rx.len;
    bus->rx_offset = evt->data.rx.offset;
    bus->rx_status = 0;
    k_event_post(&uart_servo_bus_event, RX_DONE);
    break;

  case UART_RX_STOPPED:
    LOG_WRN("RX STOPPED");
    bus->rx_len = evt->data.rx_stop.data.len;
    bus->rx_status = 0;
    k_event_post(&uart_servo_bus_event, RX_DONE);
    // k_sem_give(&bus->rx_done);
    break;

  case UART_RX_BUF_REQUEST:
    LOG_WRN("RX BUF REQUEST");
    LOG_HEXDUMP_DBG(bus->cur_rx_buf, sizeof(bus->cur_rx_buf), "RX Buf: ");
    if (bus->next_rx_buf == NULL) {
      bus->next_rx_buf = bus->rx_buf2;
      err = uart_rx_buf_rsp(dev, bus->next_rx_buf, sizeof(bus->rx_buf2));
      if (err < 0) {
        LOG_ERR("Failed to provide second buffer: %s", strerror(-err));
      }
    }
    break;
  case UART_RX_BUF_RELEASED:
    LOG_WRN("RX BUF RELEASED");
    LOG_HEXDUMP_DBG(bus->cur_rx_buf, sizeof(bus->cur_rx_buf), "RX Buf: ");
    LOG_HEXDUMP_DBG(bus->next_rx_buf, sizeof(bus->next_rx_buf),
                    "RX Buf next: ");
    if (evt->data.rx_buf.buf == bus->rx_buf) {
      bus->cur_rx_buf = bus->rx_buf2;
    } else if (evt->data.rx_buf.buf == bus->rx_buf2) {
      bus->cur_rx_buf = bus->rx_buf;
    }
    bus->next_rx_buf = NULL;
    break;
  case UART_RX_DISABLED:
    LOG_WRN("RX DISABLED!!!");
    break;
  }
}

static int st3215_bus_tx(const struct device *dev, const uint8_t *data,
                         size_t len) {
  int ret;
  struct st3215_bus *bus = &__bus_state;
  LOG_WRN("TX SEMAPHORE AWAIT");
  k_sem_take(&bus->tx_done, K_FOREVER);
  LOG_WRN("TX SEMAPHORE TAKEN");
  memset(bus->tx_buf, 0, sizeof(bus->tx_buf));
  memcpy(bus->tx_buf, data, len);
  bus->tx_status = -EAGAIN;

  ret = uart_tx(bus->uart_dev, bus->tx_buf, len, SYS_FOREVER_US);
  if (ret < 0) {
    LOG_ERR("uart_tx failed: %d", ret);
    k_sem_give(&bus->tx_done);
    return ret;
  }

  LOG_WRN("WAITING TX END EVENT");
  k_event_wait_safe(&uart_servo_bus_event, TX_DONE, false, K_FOREVER);
  LOG_WRN("TX END EVENT RECEIVED");

  if (bus->tx_status < 0) {
    LOG_ERR("TX failed: %d", bus->tx_status);
    k_sem_give(&bus->tx_done);
    return bus->tx_status;
  }
  LOG_WRN("TX SEMAPHORE RELEASED");
  k_sem_give(&bus->tx_done);

  return 0;
}

static int st3215_bus_rx(const struct device *dev, uint32_t timeout_ms) {
  int ret;
  struct st3215_bus *bus = &__bus_state;
  struct st3215_data *data = dev->data;

  k_sem_take(&bus->rx_done, K_FOREVER);
  LOG_WRN("RX SEMAPHORE TAKEN");
  bus->rx_len = 0;
  bus->rx_status = -EAGAIN;

  LOG_WRN("RX ENABLE");
  ret = uart_rx_enable(bus->uart_dev, bus->rx_buf, sizeof(bus->rx_buf),
                       timeout_ms * 1000);
  LOG_WRN("RX ENABLE DONE");

  if (ret < 0) {
    LOG_ERR("uart_rx_enable failed: %d", ret);
    k_sem_give(&bus->rx_done);
    return ret;
  }
  LOG_WRN("RX DONE AWAIT");
  k_event_wait_safe(&uart_servo_bus_event, RX_DONE, false, K_FOREVER);
  LOG_WRN("RX DONE RCVED");

  if (bus->rx_status < 0) {
    LOG_ERR("RX failed: %d", bus->rx_status);
    uart_rx_disable(bus->uart_dev);
    k_sem_give(&bus->rx_done);
    return bus->rx_status;
  }

  if (bus->rx_len < 6) {
    LOG_ERR("RX too short: %zu bytes", bus->rx_len);
    uart_rx_disable(bus->uart_dev);
    k_sem_give(&bus->rx_done);
    return -EIO;
  }

  memcpy(data->rx_buff + bus->rx_offset, bus->cur_rx_buf,
         sizeof(bus->rx_buf) - bus->rx_offset);
  data->rx_len = bus->rx_len;
  LOG_WRN("TX SEMAPHORE RELEASED");
  k_sem_give(&bus->rx_done);

  uint8_t computed_cs =
      st3215_compute_checksum(data->rx_buff + 2, data->rx_len - 3);
  if (data->rx_buff[data->rx_len - 1] != computed_cs) {
    LOG_ERR("Checksum mismatch: expected 0x%02X, got 0x%02X", computed_cs,
            data->rx_buff[data->rx_len - 1]);
    // uart_rx_disable(bus->uart_dev);
    return -EIO;
  }

  // uart_rx_disable(bus->uart_dev);

  return bus->rx_len;
}

static int st3215_send_packet(const struct device *dev, uint8_t servo_id,
                              uint8_t instruction, const uint8_t *params,
                              size_t param_len) {
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

  LOG_DBG("TX: %02X %02X %02X %02X %02X", packet[0], packet[1], packet[2],
          packet[3], packet[4]);
  if (param_len > 0) {
    LOG_HEXDUMP_DBG(params, param_len, "TX params:");
  }

  return st3215_bus_tx(dev, packet, packet_len);
}

static int st3215_read_response(const struct device *dev, uint32_t timeout_ms) {
  return st3215_bus_rx(dev, timeout_ms);
}

static int st3215_enable_torque(const struct device *dev, uint8_t servo_id) {
  uint8_t params[2] = {ST3215_REG_TORQUE_ENABLE, 1};

  return st3215_send_packet(dev, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_disable_torque(const struct device *dev, uint8_t servo_id) {
  uint8_t params[2] = {ST3215_REG_TORQUE_ENABLE, 0};

  return st3215_send_packet(dev, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_set_status_return_level(const struct device *dev,
                                          uint8_t servo_id) {
  uint8_t params[2] = {ST3215_REG_STATUS_RETURN, 1};

  return st3215_send_packet(dev, servo_id, ST3215_INST_WRITE, params, 2);
}

static int st3215_set_operating_mode(const struct device *dev, uint8_t servo_id) {
  uint8_t params[2] = {ST3215_REG_OPERATING_MODE, 0};

  return st3215_send_packet(dev, servo_id, ST3215_INST_WRITE, params, 2);
}

// static int st3215_unlock_eeprom(struct device *dev, uint8_t servo_id) {
//   uint8_t params[2] = {ST3215_REG_LOCK, 0};

//   return st3215_send_packet(dev, servo_id, ST3215_INST_WRITE, params, 2);
// }

// static int st3215_lock_eeprom(struct device *dev, uint8_t servo_id) {
//   uint8_t params[2] = {ST3215_REG_LOCK, 1};

//   return st3215_send_packet(dev, servo_id, ST3215_INST_WRITE, params, 2);
// }

static int st3215_set_position(const struct device *dev, int32_t angle_mdeg) {
  const struct st3215_config *config = dev->config;
  struct st3215_data *data = dev->data;
  struct st3215_bus *bus = &__bus_state;

  if (angle_mdeg < 0) {
    angle_mdeg = 0;
  }
  if (angle_mdeg > 360000) {
    angle_mdeg = 360000;
  }

  uint16_t raw_position =
      (uint16_t)((angle_mdeg * ST3215_POSITION_MAX) / 360000);

  uint8_t params[7];
  params[0] = ST3215_REG_TARGET_POSITION;
  params[1] = (uint8_t)(raw_position & 0xFF);
  params[2] = (uint8_t)((raw_position >> 8) & 0xFF);
  params[3] = (uint8_t)(data->time_ms & 0xFF);
  params[4] = (uint8_t)((data->time_ms >> 8) & 0xFF);
  params[5] = (uint8_t)(data->speed & 0xFF);
  params[6] = (uint8_t)((data->speed >> 8) & 0xFF);

  k_mutex_lock(&bus->lock, K_FOREVER);
  int ret =
      st3215_send_packet(dev, config->servo_id, ST3215_INST_WRITE, params, 7);
  k_mutex_unlock(&bus->lock);

  return ret;
}

static int st3215_get_position(const struct device *dev, int32_t *angle_mdeg) {
  const struct st3215_config *config = dev->config;
  struct st3215_bus *bus = &__bus_state;
  uint8_t read_params[2] = {ST3215_REG_CURRENT_POSITION, 2};

  k_mutex_lock(&bus->lock, K_FOREVER);

  int ret = st3215_send_packet(dev, config->servo_id, ST3215_INST_READ,
                               read_params, 2);
  if (ret < 0) {
    k_mutex_unlock(&bus->lock);
    return ret;
  }

  ret = st3215_read_response(dev, 50);
  k_mutex_unlock(&bus->lock);

  if (ret < 0) {
    LOG_ERR("Failed to read position response");
    return ret;
  }

  uint16_t raw_position = bus->rx_buf[5] | (bus->rx_buf[6] << 8);
  *angle_mdeg = (int32_t)((raw_position * 360000) / ST3215_POSITION_MAX);

  LOG_DBG("Raw position: %u, angle: %d.%03u deg", raw_position,
          (*angle_mdeg / 1000), (*angle_mdeg % 1000));

  return 0;
}

static int st3215_set_speed(const struct device *dev, uint16_t speed) {
  struct st3215_data *data = dev->data;
  data->speed = speed;
  LOG_DBG("Speed set to %u", speed);
  return 0;
}

static int st3215_set_time(const struct device *dev, uint16_t time_ms) {
  struct st3215_data *data = dev->data;
  data->time_ms = time_ms;
  LOG_DBG("Time set to %u ms", time_ms);
  return 0;
}

static int st3215_ping(const struct device *dev) {
  const struct st3215_config *config = dev->config;
  struct st3215_bus *bus = &__bus_state;

  k_mutex_lock(&bus->lock, K_FOREVER);

  int ret =
      st3215_send_packet(dev, config->servo_id, ST3215_INST_PING, NULL, 0);
  if (ret < 0) {
    k_mutex_unlock(&bus->lock);
    return ret;
  }

  ret = st3215_read_response(dev, 100);
  k_mutex_unlock(&bus->lock);

  if (ret < 0) {
    LOG_ERR("Ping failed: no response");
    return ret;
  }

  uint8_t status = bus->rx_buf[4];
  LOG_INF("Ping response: servo status = 0x%02X", status);

  return (status == 0) ? 0 : -EIO;
}

static int st3215_enable_torque_api(const struct device *dev) {
  const struct st3215_config *config = dev->config;
  struct st3215_bus *bus = &__bus_state;

  k_mutex_lock(&bus->lock, K_FOREVER);
  int ret = st3215_enable_torque(dev, config->servo_id);
  k_mutex_unlock(&bus->lock);

  return ret;
}

static int st3215_disable_torque_api(const struct device *dev) {
  const struct st3215_config *config = dev->config;
  struct st3215_bus *bus = &__bus_state;

  k_mutex_lock(&bus->lock, K_FOREVER);
  int ret = st3215_disable_torque(dev, config->servo_id);
  k_mutex_unlock(&bus->lock);

  return ret;
}

static int st3215_get_status(const struct device *dev, uint8_t *status) {
  const struct st3215_config *config = dev->config;
  struct st3215_bus *bus = &__bus_state;
  uint8_t read_params[2] = {ST3215_REG_SERVO_STATUS, 1};

  k_mutex_lock(&bus->lock, K_FOREVER);

  int ret = st3215_send_packet(dev, config->servo_id, ST3215_INST_READ,
                               read_params, 2);
  if (ret < 0) {
    k_mutex_unlock(&bus->lock);
    return ret;
  }

  ret = st3215_read_response(dev, 100);
  k_mutex_unlock(&bus->lock);

  if (ret < 0) {
    LOG_ERR("Failed to read status response (ID=%u)", config->servo_id);
    return ret;
  }

  *status = bus->rx_buf[5];

  if (*status & BIT(0))
    LOG_WRN("Servo %u: Voltage error", config->servo_id);
  if (*status & BIT(1))
    LOG_WRN("Servo %u: Temperature error", config->servo_id);
  if (*status & BIT(2))
    LOG_WRN("Servo %u: Overload error", config->servo_id);
  if (*status & BIT(3))
    LOG_WRN("Servo %u: Encoder error", config->servo_id);
  if (*status & BIT(4))
    LOG_WRN("Servo %u: Overcurrent error", config->servo_id);
  if (*status & BIT(5))
    LOG_WRN("Servo %u: Angle limit exceeded", config->servo_id);

  LOG_DBG("Servo %u status: 0x%02X", config->servo_id, *status);

  return 0;
}

static int st3215_init(const struct device *dev) {

  const struct st3215_config *config = dev->config;
  struct st3215_data *data = dev->data;
  struct st3215_bus *bus = &__bus_state;
  bus->uart_dev = config->uart_dev;

  if (!device_is_ready(bus->uart_dev)) {
    LOG_ERR("UART device not ready");
    return -ENODEV;
  }

  bus->cur_rx_buf = bus->rx_buf;

  k_mutex_init(&bus->lock);

  k_sem_init(&bus->tx_done, 1, 1);
  k_sem_init(&bus->rx_done, 1, 1);

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

  ret = st3215_set_status_return_level(dev, config->servo_id);
  if (ret < 0) {
    LOG_ERR("Failed to set status return level (ID=%u): %d", config->servo_id,
            ret);
  }
  k_usleep(1000);

  ret = st3215_set_operating_mode(dev, config->servo_id);
  if (ret < 0) {
    LOG_ERR("Failed to set operating mode to servo mode (ID=%u): %d",
            config->servo_id, ret);
  }
  k_usleep(1000);

  ret = st3215_enable_torque(dev, config->servo_id);
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
  static struct st3215_data st3215_data_##n;                                   \
  static const struct st3215_config st3215_config_##n = {                      \
      .uart_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                            \
      .servo_id = DT_INST_PROP(n, servo_id),                                   \
      .speed = DT_INST_PROP_OR(n, speed, 0),                                   \
      .time_ms = DT_INST_PROP_OR(n, time_ms, 0),                               \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, st3215_init, NULL, &st3215_data_##n,                \
                        &st3215_config_##n, POST_KERNEL,                       \
                        CONFIG_SERVO_INIT_PRIORITY, &st3215_api);

DT_INST_FOREACH_STATUS_OKAY(ST3215_DEFINE)
