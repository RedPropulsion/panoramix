#include "ms5611.h"
#include "ms5611_bus.h"

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/sqe.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/check.h>

#ifdef CONFIG_SENSOR_MS5611_ASYNC
#include <zephyr/drivers/sensor_clock.h>
#endif

LOG_MODULE_REGISTER(ms5611, CONFIG_SENSOR_LOG_LEVEL);

static int ms5611_reset(const struct ms5611_bus *bus) {
  int ret = ms5611_command_write_rtio(bus, COMMAND_RESET);
  if (ret < 0) {
    return ret;
  }

  k_sleep(K_MSEC(MS5611_RESET_DELAY_MS));

  return 0;
}

static inline void ms5611_compensate_raw(const uint8_t *raw_d1,
                                         const uint8_t *raw_d2,
                                         const uint16_t *coeffs,
                                         int32_t *temp_centideg,
                                         int32_t *press_mbar) {
  uint64_t d1 = sys_get_be24(raw_d1);
  uint64_t d2 = sys_get_be24(raw_d2);

  uint64_t dt = d2 - ((uint64_t)coeffs[4] << 8);
  uint64_t temp = 2000 + ((dt * coeffs[5]) >> 23);

  uint64_t off =
      ((uint64_t)coeffs[1] << 16) + (((uint64_t)coeffs[3] * dt) >> 7);
  uint64_t sens =
      ((uint64_t)coeffs[0] << 15) + (((uint64_t)coeffs[2] * dt) >> 8);
  uint64_t p = (((d1 * sens) >> 21) - off) >> 15;

  *temp_centideg = (int32_t)temp;
  *press_mbar = (int32_t)p;

  LOG_DBG("Compensate output: %d c^C, %d mBar", *temp_centideg, *press_mbar);
}

static int ms5611_init(const struct device *dev) {
  CHECKIF(dev == NULL) { return -EINVAL; }

  struct ms5611_data *data = (struct ms5611_data *)dev->data;
  struct ms5611_config *cfg = (struct ms5611_config *)dev->config;
  int ret;

  ret = ms5611_reset(&cfg->bus);
  if (ret < 0) {
    LOG_ERR("Failed to reset device: %s", strerror(-ret));
    return ret;
  }

  // MSB first!
  uint16_t raw_coeffs[6];
  for (int i = 0; i < 6; i++) {
    ret = ms5611_PROM_read_rtio(&cfg->bus, i + 1, &raw_coeffs[i]);
    if (ret < 0) {
      LOG_ERR("Failed to read PROM (offset=%d): %s", i + 1, strerror(-ret));
      return ret;
    }

    data->coeffs[i] = sys_be16_to_cpu(raw_coeffs[i]);
  }

  LOG_HEXDUMP_DBG(data->coeffs, sizeof data->coeffs, "Loaded coefficients");

  return 0;
}

static int ms5611_sample_fetch(const struct device *dev,
                               enum sensor_channel chan) {
  CHECKIF(dev == NULL) { return -EINVAL; }

  struct ms5611_data *data = (struct ms5611_data *)dev->data;
  struct ms5611_config *cfg = (struct ms5611_config *)dev->config;
  int ret;
  uint8_t d1_buf[3];
  uint8_t d2_buf[3];

  ret = ms5611_command_write_rtio(&cfg->bus, COMMAND_CONVERT_D1);
  if (ret < 0) {
    return ret;
  }
  k_sleep(K_MSEC(MS5611_MEASUREMENT_DELAY_MS));
  ret = ms5611_ADC_read_rtio(&cfg->bus, d1_buf);
  if (ret < 0) {
    LOG_ERR("Couldn't perform ADC read: %s", strerror(-ret));
    return ret;
  }

  LOG_HEXDUMP_DBG(d1_buf, sizeof d1_buf, "D1");

  ret = ms5611_command_write_rtio(&cfg->bus, COMMAND_CONVERT_D2);
  if (ret < 0) {
    return ret;
  }
  k_sleep(K_MSEC(MS5611_MEASUREMENT_DELAY_MS));
  ret = ms5611_ADC_read_rtio(&cfg->bus, d2_buf);
  if (ret < 0) {
    return ret;
  }

  LOG_HEXDUMP_DBG(d2_buf, sizeof d2_buf, "D2");

  int32_t temp, press;
  ms5611_compensate_raw(d1_buf, d2_buf, data->coeffs, &temp, &press);
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

  int32_t temp, press;
  ms5611_compensate_raw(edata->rx_d1, edata->rx_d2, edata->sensor_data->coeffs,
                        &temp, &press);

  struct sensor_q31_data *out = data_out;

  out->header.base_timestamp_ns = edata->header.timestamp;
  out->header.reading_count = 1;
  out->shift = 7;

  switch (chan_spec.chan_type) {
  case SENSOR_CHAN_AMBIENT_TEMP:
    if (!edata->flags.has_temp) {
      return -ENODATA;
    }
    // temp is in centi-degrees Celsius.
    // sensor_q31 expects degrees Celsius, so divide by 100 after q31 encoding
    out->readings[0].temperature =
        ((int64_t)temp * (INT64_C(1) << (31 - out->shift)) / 100);
    break;
  case SENSOR_CHAN_PRESS:
    if (!edata->flags.has_press) {
      return -ENODATA;
    }
    // press is in mBar (hPa).
    // sensor_q31 expects kPa, so divide by 10 after q31 encoding
    out->readings[0].value =
        ((int64_t)press * (INT64_C(1) << (31 - out->shift))) / 10;
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
  struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)arg;

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
  struct ms5611_config *cfg = (struct ms5611_config *)dev->config;

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

  int ret;
  ret = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, (uint8_t **)&edata,
                        &buf_len);
  if (ret < 0 || buf_len < min_buf_len || !edata) {
    LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
    rtio_iodev_sqe_err(iodev_sqe, ret);
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
  ret = sensor_clock_get_cycles(&cycles);
  if (ret < 0) {
    LOG_ERR("Failed to get sensor clock cycles");
    rtio_iodev_sqe_err(iodev_sqe, ret);
    return;
  }
  edata->header.timestamp = sensor_clock_cycles_to_ns(cycles);
  edata->sensor_data = data;

  struct rtio_sqe *current_sqe;

  ms5611_prep_command_write_rtio_async(&cfg->bus, COMMAND_CONVERT_D1,
                                       &current_sqe);
  if (ret < 0) {
    LOG_ERR("Couldn't perform command write: %s", strerror(-ret));
    rtio_iodev_sqe_err(iodev_sqe, ret);
    return;
  }
  current_sqe->flags |= RTIO_SQE_CHAINED;

  current_sqe = rtio_sqe_acquire(cfg->bus.rtio.ctx);
  if (!current_sqe) {
    LOG_ERR("Failed to acquire SQE");
    rtio_sqe_drop_all(cfg->bus.rtio.ctx);
    rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
    return;
  }
  rtio_sqe_prep_delay(current_sqe, K_MSEC(MS5611_MEASUREMENT_DELAY_MS), NULL);
  current_sqe->flags |= RTIO_SQE_CHAINED;

  ret = ms5611_prep_ADC_read_rtio_async(&cfg->bus, edata->rx_d1, &current_sqe);
  if (ret < 0) {
    LOG_ERR("Couldn't perform ADC read: %s", strerror(-ret));
    rtio_iodev_sqe_err(iodev_sqe, ret);
    return;
  }
  current_sqe->flags |= RTIO_SQE_CHAINED;

  ms5611_prep_command_write_rtio_async(&cfg->bus, COMMAND_CONVERT_D2,
                                       &current_sqe);
  if (ret < 0) {
    LOG_ERR("Couldn't perform command write: %s", strerror(-ret));
    rtio_iodev_sqe_err(iodev_sqe, ret);
    return;
  }
  current_sqe->flags |= RTIO_SQE_CHAINED;

  current_sqe = rtio_sqe_acquire(cfg->bus.rtio.ctx);
  if (!current_sqe) {
    LOG_ERR("Failed to acquire SQE");
    rtio_sqe_drop_all(cfg->bus.rtio.ctx);
    rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
    return;
  }
  rtio_sqe_prep_delay(current_sqe, K_MSEC(MS5611_MEASUREMENT_DELAY_MS), NULL);
  current_sqe->flags |= RTIO_SQE_CHAINED;

  ret = ms5611_prep_ADC_read_rtio_async(&cfg->bus, edata->rx_d2, &current_sqe);
  if (ret < 0) {
    LOG_ERR("Couldn't perform ADC read: %s", strerror(-ret));
    rtio_iodev_sqe_err(iodev_sqe, ret);
    return;
  }
  current_sqe->flags |= RTIO_SQE_CHAINED;

  struct rtio_sqe *complete_sqe = rtio_sqe_acquire(cfg->bus.rtio.ctx);
  if (!complete_sqe) {
    LOG_ERR("Failed to acquire completion SQE");
    rtio_sqe_drop_all(cfg->bus.rtio.ctx);
    rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
    return;
  }

  // We do not emit a completion queue event on the sensor RTIO, but we instead
  // push it in the user RTIO in the callback function.
  rtio_sqe_prep_callback_no_cqe(complete_sqe, ms5611_complete_result, iodev_sqe,
                                NULL);

  rtio_submit(cfg->bus.rtio.ctx, 0);
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

// Defines the sensor instance.
//
// SPI_DT_IODEV_DEFINE creates the device for the RTIO context.
//
// RTIO_DEFINE creates the rtio context, by specifying the sqe and cqe number.
//   - The sqe number should be enough to contain the maximum number of sqe
//     operations in a single "operation". Otherwise the device would return
//     -ENOMEM.
//   - The cqe number is not relevant, as completion event area automatically
//     used to  chain sqes, and the cqe which concludes opereations is forwarded
//     to the RTIO_CONTEXT of the caller of the sensor operations (the overflow
//     should be automatically handled by the system).
//
// The RTIO context is instantiated even if the SENSOR_ASYNC_API is disabled, as
// it enables more code reusability at the cost of a minimal overhead.
#define MS5611_INIT(inst)                                                      \
  SPI_DT_IODEV_DEFINE(ms5611_iodev_##inst, DT_DRV_INST(inst),                  \
                      MS5611_SPI_OPERATION);                                   \
                                                                               \
  RTIO_DEFINE(ms5611_rtio_ctx_##inst, 16, 1);                                  \
                                                                               \
  static struct ms5611_data ms5611_data_##inst = {0};                          \
  static const struct ms5611_config ms5611_config_##inst = {                   \
      .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),   \
      .bus = {.rtio = {.ctx = &ms5611_rtio_ctx_##inst,                         \
                       .iodev = &ms5611_iodev_##inst}}};                       \
                                                                               \
  SENSOR_DEVICE_DT_INST_DEFINE(                                                \
      inst, ms5611_init, NULL, &ms5611_data_##inst, &ms5611_config_##inst,     \
      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &ms5611_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MS5611_INIT)
