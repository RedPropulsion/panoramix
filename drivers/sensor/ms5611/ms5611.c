#include <string.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/check.h>

#define DT_DRV_COMPAT ams_ms5611

#define COMMAND_PROM_READ 0xA0
#define COMMAND_ADC_READ 0x00
#define COMMAND_CONVERT_D1 0x40
#define COMMAND_CONVERT_D2 0x50
#define COMMAND_RESET 0x1E

LOG_MODULE_REGISTER(ms5611, CONFIG_SENSOR_LOG_LEVEL);

struct ms5611_sample {
  int32_t pressure;
  int32_t temperature;
};

struct ms5611_data {
  uint16_t coeffs[6];
  struct ms5611_sample last_sample;
};

struct ms5611_config {
  struct spi_dt_spec spi;
};

static int ms5611_read_PROM(struct spi_dt_spec *spi, uint8_t offset,
                            uint16_t *result) {
  uint8_t command = COMMAND_PROM_READ | (offset << 1);
  uint8_t tx_buf[3] = {command, 0x00, 0x00};
  uint8_t rx_buf[3] = {0};

  LOG_DBG("Reading PROM offset %d, command byte: 0x%02X", offset, command);

  const struct spi_buf tx = {.buf = tx_buf, .len = 3};
  const struct spi_buf rx = {.buf = rx_buf, .len = 3};
  const struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1};
  const struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};

  int ret = spi_transceive_dt(spi, &tx_bufs, &rx_bufs);
  if (ret < 0) {
    LOG_ERR("Failed to read PROM (offset=%d): %d (%s)", offset, ret,
            strerror(-ret));
    return ret;
  }

  *result = sys_get_be16(&rx_buf[1]);
  LOG_DBG("PROM[%d] = 0x%04X", offset, *result);
  return 0;
}

static int ms5611_read_ADC(const struct spi_dt_spec *spi, uint32_t *result) {
  uint8_t tx_buf[4] = {COMMAND_ADC_READ, 0x0, 0x0, 0x0};
  uint8_t rx_buf[4] = {0};

  const struct spi_buf tx = {.buf = tx_buf, .len = 4};
  const struct spi_buf rx = {.buf = rx_buf, .len = 4};
  const struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1};
  const struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1};

  int ret = spi_transceive_dt(spi, &tx_bufs, &rx_bufs);
  if (ret < 0) {
    LOG_ERR("Failed to read ADC: %d (%s)", ret, strerror(-ret));
    return ret;
  }

  LOG_HEXDUMP_INF(&rx_buf[1], 3, "ADC: ");

  // Discard first byte
  *result = sys_get_be24(&rx_buf[1]);
  return 0;
}

static int ms5611_write(const struct spi_dt_spec *spi, uint8_t payload) {
  const struct spi_buf tx_buf = {.buf = &payload, .len = 1};
  const struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

  int ret = spi_write_dt(spi, &tx_bufs);
  if (ret < 0) {
    LOG_ERR("Failed to write command %X: %d (%s)", payload, ret,
            strerror(-ret));
    return ret;
  }

  return 0;
}

static int ms5611_reset(const struct spi_dt_spec *spi) {
  struct spi_dt_spec spi_lock = *spi;
  int ret;

  spi_lock.config.operation |= SPI_LOCK_ON | SPI_HOLD_ON_CS;

  ret = ms5611_write(&spi_lock, COMMAND_RESET);
  if (ret < 0) {
    return ret;
  }

  k_sleep(K_MSEC(10));

  ret = spi_release_dt(&spi_lock);
  if (ret < 0) {
    LOG_ERR("Failed to release CS: %d (%s)", ret, strerror(-ret));
    return ret;
  }

  return 0;
}

static void ms5611_compensate(uint32_t rawPressure, uint32_t rawTemperature,
                              struct ms5611_data *data) {
  uint64_t d1 = rawPressure;
  uint64_t d2 = rawTemperature;

  uint64_t dt = d2 - (data->coeffs[4] << 8);
  uint64_t temp = 2000 + (dt * data->coeffs[5] >> 23);

  uint64_t off = (data->coeffs[1] << 16) + ((data->coeffs[3] * dt) >> 7);
  uint64_t sens = (data->coeffs[0] << 15) + ((data->coeffs[2] * dt) >> 8);

  uint64_t p = (((d1 * sens) >> 21) - off) >> 15;

  data->last_sample.temperature = temp;
  data->last_sample.pressure = p;
}

static int ms5611_init(const struct device *dev) {
  CHECKIF(dev == NULL) { return -EINVAL; }

  struct ms5611_data *data = (struct ms5611_data *)dev->data;
  struct ms5611_config *conf = (struct ms5611_config *)dev->config;
  int ret;

  while (!spi_is_ready_dt(&conf->spi)) {
    LOG_ERR("SPI device not ready");
  }

  ret = ms5611_reset(&conf->spi);
  if (ret < 0) {
    LOG_ERR("Failed to reset device: %d (%s)", ret, strerror(-ret));
    return ret;
  }

  for (int i = 0; i < 6; i++) {
    ret = ms5611_read_PROM(&conf->spi, i + 1, &data->coeffs[i]);
    if (ret < 0) {
      LOG_ERR("Failed to retreive coeff %d: %d (%s)", i + 1, ret,
              strerror(-ret));
      return ret;
    }
  }

  LOG_HEXDUMP_INF(data->coeffs, sizeof data->coeffs, "Loaded coefficients:");

  return 0;
}

static int ms5611_sample_fetch(const struct device *dev,
                               enum sensor_channel chan) {
  CHECKIF(dev == NULL) { return -EINVAL; }

  struct ms5611_data *data = (struct ms5611_data *)dev->data;
  struct ms5611_config *conf = (struct ms5611_config *)dev->config;
  int ret;
  uint32_t rawPressure, rawTemperature;

  ret = ms5611_write(&conf->spi, COMMAND_CONVERT_D1);
  if (ret < 0) {
    return ret;
  }
  k_sleep(K_MSEC(10));
  ret = ms5611_read_ADC(&conf->spi, &rawPressure);
  if (ret < 0) {
    return ret;
  }

  ret = ms5611_write(&conf->spi, COMMAND_CONVERT_D2);
  if (ret < 0) {
    return ret;
  }
  k_sleep(K_MSEC(10));
  ret = ms5611_read_ADC(&conf->spi, &rawTemperature);
  if (ret < 0) {
    return ret;
  }

  ms5611_compensate(rawPressure, rawTemperature, data);

  return 0;
}

static int ms5611_channel_get(const struct device *dev,
                              enum sensor_channel chan,
                              struct sensor_value *val) {
  const struct ms5611_data *data = dev->data;

  switch (chan) {
  case SENSOR_CHAN_AMBIENT_TEMP:
    val->val1 = data->last_sample.temperature / 100;
    val->val2 = data->last_sample.temperature % 100 * 10000;
    break;
  case SENSOR_CHAN_PRESS:
    val->val1 = data->last_sample.pressure / 100;
    val->val2 = data->last_sample.pressure % 100 * 10000;
    break;
  default:
    return -ENOTSUP;
  }

  return 0;
}

static int ms5611_attr_set(const struct device *dev, enum sensor_channel chan,
                           enum sensor_attribute attr,
                           const struct sensor_value *val) {
  return -1;
}

static DEVICE_API(sensor, ms5611_driver_api) = {
    .sample_fetch = ms5611_sample_fetch,
    .channel_get = ms5611_channel_get,
    .attr_set = ms5611_attr_set,
};

#define MS5611_INIT(i)                                                         \
  static struct ms5611_data ms5611_data_##i = {0};                             \
  static struct ms5611_config ms5611_config_##i = {                            \
      .spi = SPI_DT_SPEC_INST_GET(i, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),      \
  };                                                                           \
  SENSOR_DEVICE_DT_INST_DEFINE(                                                \
      i, ms5611_init, NULL, &ms5611_data_##i, &ms5611_config_##i, POST_KERNEL, \
      CONFIG_SENSOR_INIT_PRIORITY, &ms5611_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MS5611_INIT)
