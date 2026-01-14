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
  struct sensor_value pressure;
  struct sensor_value temperature;
};

struct ms5611_data {
  uint16_t coeffs[6];
  struct ms5611_sample last_sample;
};

struct ms5611_config {
  struct spi_dt_spec spi;
};

/* static int ms5611_read_PROM(struct spi_dt_spec *spi, uint8_t offset, */
/*                             uint16_t *result) { */
/*   uint8_t command = COMMAND_PROM_READ | (offset << 1); */
/*   uint8_t tx_buf[3] = {command, 0x00, 0x00}; // command + 2 dummy bytes */
/*   uint8_t rx_buf[3] = {0}; */
/**/
/*   const struct spi_buf tx = {.buf = tx_buf, .len = 3}; */
/*   const struct spi_buf rx = {.buf = rx_buf, .len = 3}; */
/*   const struct spi_buf_set tx_bufs = {.buffers = &tx, .count = 1}; */
/*   const struct spi_buf_set rx_bufs = {.buffers = &rx, .count = 1}; */
/**/
/*   int ret = spi_transceive_dt(spi, &tx_bufs, &rx_bufs); */
/*   if (ret < 0) { */
/*     LOG_ERR("Failed to read PROM (offset=%d): %d (%s)", offset, ret, */
/*             strerror(-ret)); */
/*     return ret; */
/*   } */
/**/
/*   // Data is in rx_buf[1] and rx_buf[2] (rx_buf[0] is garbage during command)
 */
/*   *result = sys_get_be16(&rx_buf[1]); */
/*   return 0; */
/* } */

static int ms5611_read_PROM(struct spi_dt_spec *spi, uint8_t offset,
                            uint16_t *result) {
  // Try WITHOUT the shift first - addresses are 0xA0, 0xA2, 0xA4...
  uint8_t command = COMMAND_PROM_READ | (offset << 1);
  uint8_t tx_buf[3] = {command, 0x00, 0x00};
  uint8_t rx_buf[3] = {0};

  LOG_INF("Reading PROM offset %d, command byte: 0x%02X", offset, command);

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

  LOG_HEXDUMP_INF(rx_buf, 3, "PROM RX:");

  *result = sys_get_be16(&rx_buf[1]);
  LOG_INF("PROM[%d] = 0x%04X", offset, *result);
  return 0;
}

static int ms5611_read_ADC(const struct spi_dt_spec *spi, uint32_t *result) {
  uint8_t command = COMMAND_ADC_READ;
  uint8_t raw_rx[3] = {0, 0, 0};

  const struct spi_buf tx_buf = {.buf = &command, .len = 1};
  const struct spi_buf rx_buf = {.buf = raw_rx, .len = 3};
  const struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
  const struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};

  int ret = spi_transceive_dt(spi, &tx_bufs, &rx_bufs);
  if (ret < 0) {
    LOG_ERR("Failed to read ADC: %d (%s)", ret, strerror(-ret));
    return ret;
  }

  *result = sys_get_be24(raw_rx);
  return 0;
}

static int ms5611_write(const struct spi_dt_spec *spi, uint8_t payload) {
  const struct spi_buf tx_buf = {.buf = &payload, .len = 1};
  const struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

  int ret = spi_write_dt(spi, &tx_bufs);
  if (ret < 0) {
    LOG_ERR("Failed to write command: %d (%s)", ret, strerror(-ret));
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

static int ms5611_init(const struct device *dev) {
  CHECKIF(dev == NULL) { return -EINVAL; }

  struct ms5611_data *drv = (struct ms5611_data *)dev->data;
  struct ms5611_config *conf = (struct ms5611_config *)dev->config;
  int ret;

  // Check if SPI is ready (this should initialize CS)
  if (!spi_is_ready_dt(&conf->spi)) {
    LOG_ERR("SPI device not ready");
    return -ENODEV;
  }

  ret = ms5611_reset(&conf->spi);
  if (ret < 0) {
    LOG_ERR("Failed to reset device: %d (%s)", ret, strerror(-ret));
    return ret;
  }

  for (int i = 0; i < 6; i++) {
    ret = ms5611_read_PROM(&conf->spi, i + 1, &drv->coeffs[i]);
    if (ret < 0) {
      LOG_ERR("Failed to retreive coeff %d: %d (%s)", i + 1, ret,
              strerror(-ret));
      return ret;
    }
  }

  LOG_HEXDUMP_INF(drv->coeffs, sizeof drv->coeffs, "Loaded coefficients:");

  return 0;
}

static int ms5611_sample_fetch(const struct device *dev,
                               enum sensor_channel chan) {
  return -1;
}

static int ms5611_channel_get(const struct device *dev,
                              enum sensor_channel chan,
                              struct sensor_value *val) {
  return -1;
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
