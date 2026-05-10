#include "ms5611.h"

#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/check.h>

#ifdef CONFIG_SENSOR_MS5611_ASYNC
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/rtio/rtio.h>
#endif

LOG_MODULE_REGISTER(ms5611, CONFIG_SENSOR_LOG_LEVEL);

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

  k_sleep(K_MSEC(MS5611_MEASUREMENT_DELAY_MS));

  ret = spi_release_dt(&spi_lock);
  if (ret < 0) {
    LOG_ERR("Failed to release CS: %d (%s)", ret, strerror(-ret));
    return ret;
  }

  return 0;
}

static inline void ms5611_compensate_raw(uint32_t rawPressure, uint32_t rawTemperature,
                                          uint16_t *coeffs,
                                          int32_t *temp_centideg, int32_t *press_pa) {
  uint64_t d1 = rawPressure;
  uint64_t d2 = rawTemperature;

  uint64_t dt = d2 - ((uint64_t)coeffs[4] << 8);
  uint64_t temp = 2000 + (dt * coeffs[5] >> 23);

  uint64_t off = ((uint64_t)coeffs[1] << 16) + ((uint64_t)coeffs[3] * dt >> 7);
  uint64_t sens = ((uint64_t)coeffs[0] << 15) + ((uint64_t)coeffs[2] * dt >> 8);
  uint64_t p = (((d1 * sens) >> 21) - off) >> 15;

  *temp_centideg = (int32_t)temp;
  *press_pa = (int32_t)p;
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
  k_sleep(K_MSEC(MS5611_MEASUREMENT_DELAY_MS));
  ret = ms5611_read_ADC(&conf->spi, &rawPressure);
  if (ret < 0) {
    return ret;
  }

  ret = ms5611_write(&conf->spi, COMMAND_CONVERT_D2);
  if (ret < 0) {
    return ret;
  }
  k_sleep(K_MSEC(MS5611_MEASUREMENT_DELAY_MS));
  ret = ms5611_read_ADC(&conf->spi, &rawTemperature);
  if (ret < 0) {
    return ret;
  }

  int32_t temp, press;
  ms5611_compensate_raw(rawPressure, rawTemperature, data->coeffs, &temp, &press);
  data->last_sample.temperature = temp;
  data->last_sample.pressure = press;

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
  return -ENOTSUP;
}

#ifdef CONFIG_SENSOR_MS5611_ASYNC

static int ms5611_decoder_get_frame_count(const uint8_t *buffer,
                                          struct sensor_chan_spec chan_spec,
                                          uint16_t *frame_count) {
  const struct ms5611_encoded_data *edata =
      (const struct ms5611_encoded_data *)buffer;

  if (chan_spec.chan_idx != 0) {
    return -ENOTSUP;
  }

  switch (chan_spec.chan_type) {
  case SENSOR_CHAN_AMBIENT_TEMP:
    *frame_count = edata->flags.has_temp ? 1 : 0;
    break;
  case SENSOR_CHAN_PRESS:
    *frame_count = edata->flags.has_press ? 1 : 0;
    break;
  default:
    return -ENOTSUP;
  }

  return *frame_count > 0 ? 0 : -ENODATA;
}

static int ms5611_decoder_get_size_info(struct sensor_chan_spec chan_spec,
                                        size_t *base_size, size_t *frame_size) {
  switch (chan_spec.chan_type) {
  case SENSOR_CHAN_AMBIENT_TEMP:
  case SENSOR_CHAN_PRESS:
    *base_size = sizeof(struct sensor_q31_data);
    *frame_size = sizeof(struct sensor_q31_sample_data);
    return 0;
  default:
    return -ENOTSUP;
  }
}

static int ms5611_decoder_decode(const uint8_t *buffer,
                                 struct sensor_chan_spec chan_spec,
                                 uint32_t *fit, uint16_t max_count,
                                 void *data_out) {
  const struct ms5611_encoded_data *edata =
      (const struct ms5611_encoded_data *)buffer;

  if (*fit != 0 || max_count < 1) {
    return -EINVAL;
  }

  uint16_t *coeffs = edata->sensor_data->coeffs;

  uint32_t rawPressure = sys_get_be24(&edata->rx_d1[1]);
  uint32_t rawTemperature = sys_get_be24(&edata->rx_d2[1]);

  int32_t temp, press;
  ms5611_compensate_raw(rawPressure, rawTemperature, coeffs, &temp, &press);

  struct sensor_q31_data *out = data_out;

  out->header.base_timestamp_ns = edata->header.timestamp;
  out->header.reading_count = 1;
  out->shift = -15;

  switch (chan_spec.chan_type) {
  case SENSOR_CHAN_AMBIENT_TEMP:
    if (!edata->flags.has_temp) {
      return -ENODATA;
    }
    // temp is in centi-degrees (e.g., 2500 = 25.00°C)
    // Convert to Q31: actual °C * 2^15 = centi-degrees * 32768 / 10000
    out->readings[0].value = (temp * 32768) / 10000;
    break;
  case SENSOR_CHAN_PRESS:
    if (!edata->flags.has_press) {
      return -ENODATA;
    }
    // pressure is in Pa (e.g., 101325 = 1013.25 hPa = 101.325 kPa)
    // sensor_q31 expects kPa, so convert: Pa / 1000 = kPa
    // Then convert to Q31: kPa * 2^15 = Pa * 32768 / 1000
    out->readings[0].value = ((int64_t)press * 32768) / 1000;
    break;
  default:
    return -EINVAL;
  }

  *fit = 1;
  return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
    .get_frame_count = ms5611_decoder_get_frame_count,
    .get_size_info = ms5611_decoder_get_size_info,
    .decode = ms5611_decoder_decode,
};

int ms5611_get_decoder(const struct device *dev,
                       const struct sensor_decoder_api **decoder) {
  ARG_UNUSED(dev);
  *decoder = &SENSOR_DECODER_NAME();
  return 0;
}

static void ms5611_complete_result(struct rtio *ctx, const struct rtio_sqe *sqe,
                                   int result, void *arg) {
  ARG_UNUSED(arg);

  // Documentation says they can be safely cast to and from each other.
  // ref:
  // https://docs.zephyrproject.org/latest/doxygen/html/structrtio__iodev__sqe.html
  struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)sqe;

  if (result < 0) {
    rtio_iodev_sqe_err(iodev_sqe, result);
    return;
  }

  // Raw ADC data is already in rx_d1 and rx_d2 buffers from SPI operations.
  // Compensation is done in the decoder to keep separation of concerns.
  rtio_iodev_sqe_ok(iodev_sqe, 0);
}

static void ms5611_submit(const struct device *dev,
                          struct rtio_iodev_sqe *iodev_sqe) {
  const struct sensor_read_config *read_cfg = iodev_sqe->sqe.iodev->data;
  struct ms5611_data *data = (struct ms5611_data *)dev->data;

  // Validate requested channels
  const struct sensor_chan_spec *const channels = read_cfg->channels;
  const size_t num_channels = read_cfg->count;

  for (size_t i = 0; i < num_channels; i++) {
    switch (channels[i].chan_type) {
    case SENSOR_CHAN_AMBIENT_TEMP:
    case SENSOR_CHAN_PRESS:
    case SENSOR_CHAN_ALL:
      break;
    default:
      rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
      return;
    }
  }

  // Init an rx buffer for rtio operations
  static const uint32_t min_buf_len = sizeof(struct ms5611_encoded_data);
  struct ms5611_encoded_data *edata;
  uint32_t buf_len;

  int err;
  err = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, (uint8_t **)&edata,
                        &buf_len);
  if (err < 0 || buf_len < min_buf_len || !edata) {
    LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
    rtio_iodev_sqe_err(iodev_sqe, err);
    return;
  }

  edata->flags.has_temp = 0;
  edata->flags.has_press = 0;

  for (size_t i = 0; i < num_channels; i++) {
    switch (channels[i].chan_type) {
    case SENSOR_CHAN_AMBIENT_TEMP:
      edata->flags.has_temp = 1;
      break;
    case SENSOR_CHAN_PRESS:
      edata->flags.has_press = 1;
      break;
    case SENSOR_CHAN_ALL:
      edata->flags.has_temp = 1;
      edata->flags.has_press = 1;
      break;
    default:
      break;
    }
  }

  // Get hardware timestamp
  uint64_t cycles;
  err = sensor_clock_get_cycles(&cycles);
  if (err != 0) {
    LOG_ERR("Failed to get sensor clock cycles");
    rtio_iodev_sqe_err(iodev_sqe, err);
    return;
  }
  edata->header.timestamp = sensor_clock_cycles_to_ns(cycles);
  edata->sensor_data = data;

  struct rtio *ctx = data->rtio_ctx;
  struct rtio_iodev *iodev = data->iodev;

  // Prepare buffers for conversion and reading
  struct spi_buf tx_d1_conv_buf = {.buf = (uint8_t[]){COMMAND_CONVERT_D1},
                                   .len = 1};
  struct spi_buf tx_d1_read_buf = {.buf = (uint8_t[]){COMMAND_ADC_READ},
                                   .len = 1};
  struct spi_buf rx_d1_buf = {.buf = edata->rx_d1, .len = 4};

  struct spi_buf_set tx_d1_conv_set = {.buffers = &tx_d1_conv_buf, .count = 1};
  struct spi_buf_set tx_d1_read_set = {.buffers = &tx_d1_read_buf, .count = 1};
  struct spi_buf_set rx_d1_set = {.buffers = &rx_d1_buf, .count = 1};

  struct spi_buf tx_d2_conv_buf = {.buf = (uint8_t[]){COMMAND_CONVERT_D2},
                                   .len = 1};
  struct spi_buf tx_d2_read_buf = {.buf = (uint8_t[]){COMMAND_ADC_READ},
                                   .len = 1};
  struct spi_buf rx_d2_buf = {.buf = edata->rx_d2, .len = 4};
  struct spi_buf_set tx_d2_conv_set = {.buffers = &tx_d2_conv_buf, .count = 1};
  struct spi_buf_set tx_d2_read_set = {.buffers = &tx_d2_read_buf, .count = 1};
  struct spi_buf_set rx_d2_set = {.buffers = &rx_d2_buf, .count = 1};

  struct rtio_sqe *last_sqe;

  err = spi_rtio_copy(ctx, iodev, &tx_d1_conv_set, NULL, &last_sqe);
  if (err < 0) {
    LOG_ERR("Failed to prepare SPI write D1 convert");
    rtio_iodev_sqe_err(iodev_sqe, err);
    return;
  }
  last_sqe->flags |= RTIO_SQE_CHAINED;

  last_sqe = rtio_sqe_acquire(ctx);
  if (!last_sqe) {
    LOG_ERR("Failed to acquire delay SQE");
    rtio_sqe_drop_all(ctx);
    rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
    return;
  }
  rtio_sqe_prep_delay(last_sqe, K_MSEC(MS5611_MEASUREMENT_DELAY_MS), NULL);
  last_sqe->flags |= RTIO_SQE_CHAINED;

  err = spi_rtio_copy(ctx, iodev, &tx_d1_read_set, &rx_d1_set, &last_sqe);
  if (err < 0) {
    LOG_ERR("Failed to prepare SPI read D1");
    rtio_sqe_drop_all(ctx);
    rtio_iodev_sqe_err(iodev_sqe, err);
    return;
  }
  last_sqe->flags |= RTIO_SQE_CHAINED;

  err = spi_rtio_copy(ctx, iodev, &tx_d2_conv_set, NULL, &last_sqe);
  if (err < 0) {
    LOG_ERR("Failed to prepare SPI write D2 convert");
    rtio_sqe_drop_all(ctx);
    rtio_iodev_sqe_err(iodev_sqe, err);
    return;
  }
  last_sqe->flags |= RTIO_SQE_CHAINED;

  last_sqe = rtio_sqe_acquire(ctx);
  if (!last_sqe) {
    LOG_ERR("Failed to acquire delay SQE");
    rtio_sqe_drop_all(ctx);
    rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
    return;
  }
  rtio_sqe_prep_delay(last_sqe, K_MSEC(MS5611_MEASUREMENT_DELAY_MS), NULL);
  last_sqe->flags |= RTIO_SQE_CHAINED;

  err = spi_rtio_copy(ctx, iodev, &tx_d2_read_set, &rx_d2_set, &last_sqe);
  if (err < 0) {
    LOG_ERR("Failed to prepare SPI read D2");
    rtio_sqe_drop_all(ctx);
    rtio_iodev_sqe_err(iodev_sqe, err);
    return;
  }
  last_sqe->flags |= RTIO_SQE_CHAINED;

  struct rtio_sqe *complete_sqe = rtio_sqe_acquire(ctx);
  if (!complete_sqe) {
    LOG_ERR("Failed to acquire completion SQE");
    rtio_sqe_drop_all(ctx);
    rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
    return;
  }

  rtio_sqe_prep_callback(complete_sqe, ms5611_complete_result, NULL, NULL);

  rtio_submit(ctx, 0);
}

#endif

static DEVICE_API(sensor, ms5611_driver_api) = {
    .sample_fetch = ms5611_sample_fetch,
    .channel_get = ms5611_channel_get,
    .attr_set = ms5611_attr_set,
#ifdef CONFIG_SENSOR_MS5611_ASYNC
    .submit = ms5611_submit,
    .get_decoder = ms5611_get_decoder,
#endif
};

#ifdef CONFIG_SENSOR_MS5611_ASYNC
#define MS5611_RTIO_DEFINE(inst)                                               \
  SPI_DT_IODEV_DEFINE(ms5611_iodev_##inst, DT_DRV_INST(inst),                  \
                      MS5611_SPI_OPERATION);                                   \
  RTIO_DEFINE(ms5611_rtio_ctx_##inst, 8, 8);

#define MS5611_INIT(inst)                                                      \
  MS5611_RTIO_DEFINE(inst);                                                    \
  static struct ms5611_data ms5611_data_##inst = {                             \
      .rtio_ctx = &ms5611_rtio_ctx_##inst,                                     \
      .iodev = &ms5611_iodev_##inst,                                           \
  };                                                                           \
  static const struct ms5611_config ms5611_config_##inst = {                   \
      .spi = SPI_DT_SPEC_INST_GET(inst, MS5611_SPI_OPERATION),                 \
  };                                                                           \
  SENSOR_DEVICE_DT_INST_DEFINE(                                                \
      inst, ms5611_init, NULL, &ms5611_data_##inst, &ms5611_config_##inst,     \
      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &ms5611_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MS5611_INIT)
#else
#define MS5611_INIT(inst)                                                      \
  static struct ms5611_data ms5611_data_##inst = {0};                          \
  static const struct ms5611_config ms5611_config_##inst = {                   \
      .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),   \
  };                                                                           \
  SENSOR_DEVICE_DT_INST_DEFINE(                                                \
      inst, ms5611_init, NULL, &ms5611_data_##inst, &ms5611_config_##inst,     \
      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &ms5611_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MS5611_INIT)
#endif
