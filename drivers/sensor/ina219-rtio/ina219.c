#include "ina219.h"
#include "ina219_bus.h"

#include <zephyr/drivers/i2c/rtio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/sqe.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(INA219_ASYNC, CONFIG_SENSOR_LOG_LEVEL);

static int ina219_set_config(const struct device *dev) {
  const struct ina219_config *cfg = dev->config;
  uint16_t reg_data;

  reg_data = (cfg->brng & INA219_BRNG_MASK) << INA219_BRNG_SHIFT |
             (cfg->pg & INA219_PG_MASK) << INA219_PG_SHIFT |
             (cfg->badc & INA219_ADC_MASK) << INA219_BADC_SHIFT |
             (cfg->sadc & INA219_ADC_MASK) << INA219_SADC_SHIFT |
             (cfg->mode & INA219_MODE_NORMAL);

  return ina219_reg_write_rtio(&cfg->bus, INA219_REG_CONF, reg_data);
}

static int ina219_set_calib(const struct device *dev) {
  const struct ina219_config *cfg = dev->config;
  uint16_t cal;

  cal = INA219_SCALING_FACTOR / ((cfg->r_shunt) * (cfg->current_lsb));

  return ina219_reg_write_rtio(&cfg->bus, INA219_REG_CALIB, cal);
}

static int ina219_set_msr_delay(const struct device *dev) {
  const struct ina219_config *cfg = dev->config;
  struct ina219_data *data = dev->data;

  data->msr_delay = ina219_conv_delay(cfg->badc) + ina219_conv_delay(cfg->sadc);
  return 0;
}

static int ina219_reg_field_update(const struct device *dev, uint8_t addr,
                                   uint16_t mask, uint16_t field) {
  const struct ina219_config *cfg = dev->config;
  uint16_t reg_data;
  uint8_t rx_buf[2];
  int rc;

  rc = ina219_reg_read_rtio(&cfg->bus, addr, rx_buf);
  if (rc) {
    return rc;
  }
  reg_data = sys_get_be16(rx_buf);

  reg_data = (reg_data & ~mask) | field;

  return ina219_reg_write_rtio(&cfg->bus, addr, reg_data);
}

static int ina219_init(const struct device *dev) {
  const struct ina219_config *cfg = dev->config;
  int ret;

  ret = ina219_reg_write_rtio(&cfg->bus, INA219_REG_CONF, INA219_RST);
  if (ret) {
    LOG_ERR("Could not reset device.");
    return ret;
  }

  ret = ina219_set_config(dev);
  if (ret) {
    LOG_ERR("Could not set configuration data.");
    return ret;
  }

  ret = ina219_set_calib(dev);
  if (ret) {
    LOG_DBG("Could not set calibration data.");
    return ret;
  }

  /* Set measurement delay */
  ina219_set_msr_delay(dev);

  k_sleep(K_USEC(INA219_WAIT_STARTUP));

  return 0;
}

static int ina219_sample_fetch(const struct device *dev,
                               enum sensor_channel chan) {
  struct ina219_data *data = dev->data;
  const struct ina219_config *cfg = dev->config;
  uint16_t status;
  uint8_t rx_buf[2];
  uint16_t tmp;
  int rc;

  if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_VOLTAGE &&
      chan != SENSOR_CHAN_POWER && chan != SENSOR_CHAN_CURRENT) {
    return -ENOTSUP;
  }

  /* Trigger measurement and wait for completion */
  rc = ina219_reg_field_update(dev, INA219_REG_CONF, INA219_MODE_MASK,
                               INA219_MODE_NORMAL);
  if (rc) {
    LOG_ERR("Failed to start measurement.");
    return rc;
  }

  k_sleep(K_USEC(data->msr_delay));

  rc = ina219_reg_read_rtio(&cfg->bus, INA219_REG_V_BUS, rx_buf);
  if (rc) {
    LOG_ERR("Failed to read device status.");
    return rc;
  }
  status = sys_get_be16(rx_buf);

  while (!(INA219_CNVR_RDY(status))) {
    rc = ina219_reg_read_rtio(&cfg->bus, INA219_REG_V_BUS, rx_buf);
    if (rc) {
      LOG_ERR("Failed to read device status.");
      return rc;
    }
    status = sys_get_be16(rx_buf);
    k_sleep(K_USEC(INA219_WAIT_MSR_RETRY));
  }

  /* Check for overflow */
  if (INA219_OVF_STATUS(status)) {
    LOG_WRN("Power and/or Current calculations are out of range.");
  }

  if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE) {

    rc = ina219_reg_read_rtio(&cfg->bus, INA219_REG_V_BUS, rx_buf);
    if (rc) {
      LOG_ERR("Error reading bus voltage.");
      return rc;
    }
    tmp = sys_get_be16(rx_buf);
    data->v_bus = INA219_VBUS_GET(tmp);
  }

  if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POWER) {

    rc = ina219_reg_read_rtio(&cfg->bus, INA219_REG_POWER, rx_buf);
    if (rc) {
      LOG_ERR("Error reading power register.");
      return rc;
    }
    data->power = sys_get_be16(rx_buf);
  }

  if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_CURRENT) {

    rc = ina219_reg_read_rtio(&cfg->bus, INA219_REG_CURRENT, rx_buf);
    if (rc) {
      LOG_ERR("Error reading current register.");
      return rc;
    }
    data->current = sys_get_be16(rx_buf);
  }

  return rc;
}

static int ina219_channel_get(const struct device *dev,
                              enum sensor_channel chan,
                              struct sensor_value *val) {
  const struct ina219_config *cfg = dev->config;
  struct ina219_data *data = dev->data;
  double tmp;
  int8_t sign = 1;

  switch (chan) {
  case SENSOR_CHAN_VOLTAGE:
    tmp = data->v_bus * INA219_V_BUS_MUL;
    break;
  case SENSOR_CHAN_POWER:
    tmp = data->power * cfg->current_lsb * INA219_POWER_MUL * INA219_SI_MUL;
    break;
  case SENSOR_CHAN_CURRENT:
    if (INA219_SIGN_BIT(data->current)) {
      data->current = ~data->current + 1;
      sign = -1;
    }
    tmp = sign * data->current * cfg->current_lsb * INA219_SI_MUL;
    break;
  default:
    LOG_DBG("Channel not supported by device!");
    return -ENOTSUP;
  }

  return sensor_value_from_double(val, tmp);
}

static int ina219_decoder_get_frame_count(const uint8_t *buffer,
                                          struct sensor_chan_spec chan_spec,
                                          uint16_t *frame_count) {
  const struct ina219_encoded_data *edata =
      (const struct ina219_encoded_data *)buffer;

  if (chan_spec.chan_idx != 0) {
    return -ENOTSUP;
  }

  switch (chan_spec.chan_type) {
  case SENSOR_CHAN_VOLTAGE:
    *frame_count = edata->flags.has_voltage ? 1 : 0;
    break;
  case SENSOR_CHAN_CURRENT:
    *frame_count = edata->flags.has_current ? 1 : 0;
    break;
  case SENSOR_CHAN_POWER:
    *frame_count = edata->flags.has_power ? 1 : 0;
    break;
  default:
    return -ENOTSUP;
  }

  return *frame_count > 0 ? 0 : -ENODATA;
}

static int ina219_decoder_get_size_info(struct sensor_chan_spec chan_spec,
                                        size_t *base_size, size_t *frame_size) {
  switch (chan_spec.chan_type) {
  case SENSOR_CHAN_VOLTAGE:
  case SENSOR_CHAN_CURRENT:
  case SENSOR_CHAN_POWER:
    *base_size = sizeof(struct sensor_q31_data);
    *frame_size = sizeof(struct sensor_q31_sample_data);
    return 0;
  default:
    return -ENOTSUP;
  }
}

static int ina219_decoder_decode(const uint8_t *buffer,
                                 struct sensor_chan_spec chan_spec,
                                 uint32_t *fit, uint16_t max_count,
                                 void *data_out) {
  const struct ina219_encoded_data *edata =
      (const struct ina219_encoded_data *)buffer;

  if (*fit != 0 || max_count < 1) {
    return -EINVAL;
  }

  struct sensor_q31_data *out = data_out;

  out->header.base_timestamp_ns = edata->header.timestamp;
  out->header.reading_count = 1;
  // TODO: adjust shift
  out->shift = 15;

  double value;

  switch (chan_spec.chan_type) {
  case SENSOR_CHAN_VOLTAGE:
    if (!edata->flags.has_voltage) {
      return -ENODATA;
    }

    uint16_t v_bus = INA219_VBUS_GET(sys_get_be16(edata->rx_buf_voltage));
    value = v_bus * INA219_V_BUS_MUL;
    break;
  default:
    return -EINVAL;
  }

  out->readings[0].value = (value * (INT64_C(1) << (31 - out->shift)));

  *fit = 1;
  return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
    .get_frame_count = ina219_decoder_get_frame_count,
    .get_size_info = ina219_decoder_get_size_info,
    .decode = ina219_decoder_decode,
};

int ina219_get_decoder(const struct device *dev,
                       const struct sensor_decoder_api **decoder) {
  ARG_UNUSED(dev);
  *decoder = &SENSOR_DECODER_NAME();
  return 0;
}

static void ina219_readings_cb(struct rtio *ctx, const struct rtio_sqe *sqe,
                               int result, void *arg) {
  struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)arg;

  // Drain CQEs
  struct rtio_cqe *cqe;
  int err = result;
  do {
    cqe = rtio_cqe_consume(ctx);
    if (cqe) {
      if (!err)
        err = cqe->result;
      rtio_cqe_release(ctx, cqe);
    }
  } while (cqe);

  if (err) {
    rtio_iodev_sqe_err(iodev_sqe, err);
  } else {
    rtio_iodev_sqe_ok(iodev_sqe, 0);
  }
}

// This callaback needs to be invoked after a status read operation, where it
// loops by creating delayed sequences which continuosly fetch and check the
// sensor status. Once the sensor is ready, it submits a read sequence.
static void ina219_check_ready_cb(struct rtio *ctx, const struct rtio_sqe *sqe,
                                  int result, void *arg) {
  struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)arg;
  struct device *dev = sqe->userdata;
  const struct ina219_config *cfg = dev->config;

  struct ina219_encoded_data *edata;
  int32_t buf_len;

  rtio_sqe_rx_buf(iodev_sqe, sizeof(struct ina219_encoded_data),
                  sizeof(struct ina219_encoded_data), (uint8_t **)&edata,
                  &buf_len);

  // Drain CQEs
  struct rtio_cqe *cqe;
  int err = result;
  do {
    cqe = rtio_cqe_consume(ctx);
    if (cqe) {
      if (!err)
        err = cqe->result;
      rtio_cqe_release(ctx, cqe);
    }
  } while (cqe);

  if (err) {
    rtio_iodev_sqe_err(iodev_sqe, err);
    return;
  }

  uint16_t status = sys_get_be16(edata->rx_buf);

  if (!(INA219_CNVR_RDY(status))) {
    struct rtio_sqe *current_sqe = rtio_sqe_acquire(ctx);
    if (!current_sqe) {
      rtio_sqe_drop_all(ctx);
      rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
      return;
    }

    rtio_sqe_prep_delay(current_sqe, K_USEC(INA219_WAIT_MSR_RETRY), NULL);
    current_sqe->flags |= RTIO_SQE_CHAINED;

    /* prep your async I2C read of INA219_REG_V_BUS into poll->status_buf */
    ina219_prep_reg_read_rtio_async(&cfg->bus, INA219_REG_V_BUS, edata->rx_buf,
                                    &current_sqe);
    current_sqe->flags |= RTIO_SQE_CHAINED;

    current_sqe = rtio_sqe_acquire(cfg->bus.rtio.ctx);
    if (!current_sqe) {
      rtio_sqe_drop_all(cfg->bus.rtio.ctx);
      rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
      return;
    }
    rtio_sqe_prep_callback_no_cqe(current_sqe, ina219_check_ready_cb, iodev_sqe,
                                  dev);
    rtio_submit(cfg->bus.rtio.ctx, 0);
    return;
  }

  if (INA219_OVF_STATUS(status)) {
    LOG_WRN("Power and/or Current calculations are out of range.");
  }

  struct rtio_sqe *current_sqe = NULL;

  if (edata->flags.has_voltage) {
    int ret = ina219_prep_reg_read_rtio_async(
        &cfg->bus, INA219_REG_V_BUS, edata->rx_buf_voltage, &current_sqe);
    if (ret < 0) {
      LOG_ERR("Could not perform voltage reading");
      rtio_iodev_sqe_err(iodev_sqe, ret);
      return;
    }
    current_sqe->flags |= RTIO_SQE_CHAINED;
  }

  if (edata->flags.has_current) {
    int ret = ina219_prep_reg_read_rtio_async(
        &cfg->bus, INA219_REG_CURRENT, edata->rx_buf_current, &current_sqe);
    if (ret < 0) {
      LOG_ERR("Could not perform current reading");
      rtio_iodev_sqe_err(iodev_sqe, ret);
      return;
    }
    current_sqe->flags |= RTIO_SQE_CHAINED;
  }

  if (edata->flags.has_power) {
    int ret = ina219_prep_reg_read_rtio_async(
        &cfg->bus, INA219_REG_POWER, edata->rx_buf_power, &current_sqe);
    if (ret < 0) {
      LOG_ERR("Could not perform power reading");
      rtio_iodev_sqe_err(iodev_sqe, ret);
      return;
    }
    current_sqe->flags |= RTIO_SQE_CHAINED;
  }

  if (current_sqe) {
    current_sqe = rtio_sqe_acquire(cfg->bus.rtio.ctx);
    if (!current_sqe) {
      rtio_sqe_drop_all(cfg->bus.rtio.ctx);
      rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
      return;
    }

    rtio_sqe_prep_callback_no_cqe(current_sqe, ina219_readings_cb, iodev_sqe,
                                  NULL);

    rtio_submit(cfg->bus.rtio.ctx, 0);
    return;
  }

  rtio_iodev_sqe_err(iodev_sqe, -ENODATA);
}

// This callaback needs to be invoked after a status read operation, where it
// sets the status bit which triggers measurement.
static void ina219_trigger_measurement_cb(struct rtio *ctx,
                                          const struct rtio_sqe *sqe,
                                          int result, void *arg) {
  struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)arg;
  struct device *dev = sqe->userdata;
  struct ina219_config *cfg = (struct ina219_config *)dev->config;
  struct ina219_data *data = (struct ina219_data *)dev->data;

  struct ina219_encoded_data *edata;
  int32_t buf_len;

  rtio_sqe_rx_buf(iodev_sqe, sizeof(struct ina219_encoded_data),
                  sizeof(struct ina219_encoded_data), (uint8_t **)&edata,
                  &buf_len);

  // Drain CQEs
  struct rtio_cqe *cqe;
  int err = result;
  do {
    cqe = rtio_cqe_consume(ctx);
    if (cqe) {
      if (!err)
        err = cqe->result;
      rtio_cqe_release(ctx, cqe);
    }
  } while (cqe);

  if (err) {
    rtio_iodev_sqe_err(iodev_sqe, err);
    return;
  }

  uint16_t status = sys_get_be16(edata->rx_buf);
  status = (status & ~INA219_MODE_MASK) | INA219_MODE_NORMAL;

  struct rtio_sqe *current_sqe;

  int ret = ina219_prep_reg_write_rtio_async(&cfg->bus, INA219_REG_CONF, status,
                                             &current_sqe);
  if (ret < 0) {
    LOG_ERR("Failed to set device config");
    rtio_iodev_sqe_err(iodev_sqe, ret);
    return;
  }
  current_sqe->flags |= RTIO_SQE_CHAINED;

  current_sqe = rtio_sqe_acquire(cfg->bus.rtio.ctx);
  if (!current_sqe) {
    LOG_ERR("Failed to acquire SQE");
    rtio_sqe_drop_all(cfg->bus.rtio.ctx);
    rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
  }
  rtio_sqe_prep_delay(current_sqe, K_USEC(data->msr_delay), NULL);
  current_sqe->flags |= RTIO_SQE_CHAINED;

  ret = ina219_prep_reg_read_rtio_async(&cfg->bus, INA219_REG_V_BUS,
                                        edata->rx_buf, &current_sqe);
  if (ret < 0) {
    LOG_ERR("Failed to read device status");
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

  rtio_sqe_prep_callback_no_cqe(complete_sqe, ina219_check_ready_cb, iodev_sqe,
                                (void *)dev);

  rtio_submit(cfg->bus.rtio.ctx, 0);
}

static void ina219_submit(const struct device *dev,
                          struct rtio_iodev_sqe *iodev_sqe) {
  const struct sensor_read_config *read_cfg = iodev_sqe->sqe.iodev->data;
  struct ina219_config *cfg = (struct ina219_config *)dev->config;

  // Validate requested channels
  const struct sensor_chan_spec *const channels = read_cfg->channels;
  const size_t num_channels = read_cfg->count;

  for (size_t i = 0; i < num_channels; i++) {
    switch (channels[i].chan_type) {
    case SENSOR_CHAN_VOLTAGE:
    case SENSOR_CHAN_POWER:
    case SENSOR_CHAN_CURRENT:
    case SENSOR_CHAN_ALL:
      break;
    default:
      rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
      return;
    }
  }

  // Init an rx buffer for rtio operations
  static const uint32_t min_buf_len = sizeof(struct ina219_encoded_data);
  struct ina219_encoded_data *edata;
  uint32_t buf_len;

  int ret;
  ret = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, (uint8_t **)&edata,
                        &buf_len);
  if (ret < 0 || buf_len < min_buf_len || !edata) {
    LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
    rtio_iodev_sqe_err(iodev_sqe, ret);
    return;
  }

  edata->flags.has_voltage = 0;
  edata->flags.has_current = 0;
  edata->flags.has_power = 0;

  for (size_t i = 0; i < num_channels; i++) {
    switch (channels[i].chan_type) {
    case SENSOR_CHAN_VOLTAGE:
      edata->flags.has_voltage = 1;
      break;
    case SENSOR_CHAN_CURRENT:
      edata->flags.has_current = 1;
      break;
    case SENSOR_CHAN_POWER:
      edata->flags.has_power = 1;
      break;
    case SENSOR_CHAN_ALL:
      edata->flags.has_voltage = 1;
      edata->flags.has_current = 1;
      edata->flags.has_power = 1;
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

  // Trigger measurement
  struct rtio_sqe *read_conf_sqe;

  ret = ina219_prep_reg_read_rtio_async(&cfg->bus, INA219_REG_CONF,
                                        edata->rx_buf, &read_conf_sqe);
  if (ret < 0) {
    LOG_ERR("Could not perform register read");
    rtio_iodev_sqe_err(iodev_sqe, ret);
    return;
  }
  read_conf_sqe->flags |= RTIO_SQE_CHAINED;

  struct rtio_sqe *complete_sqe = rtio_sqe_acquire(cfg->bus.rtio.ctx);
  if (!complete_sqe) {
    LOG_ERR("Failed to acquire completion SQE");
    rtio_sqe_drop_all(cfg->bus.rtio.ctx);
    rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
    return;
  }

  rtio_sqe_prep_callback_no_cqe(complete_sqe, ina219_trigger_measurement_cb,
                                iodev_sqe, (void *)dev);

  rtio_submit(cfg->bus.rtio.ctx, 0);
}

static DEVICE_API(sensor, ina219_api) = {.sample_fetch = ina219_sample_fetch,
                                         .channel_get = ina219_channel_get,
                                         .get_decoder = ina219_get_decoder,
                                         .submit = ina219_submit};

#define INA219_INIT(inst)                                                      \
  I2C_DT_IODEV_DEFINE(ina219_iodev_##inst, DT_DRV_INST(inst));                 \
                                                                               \
  RTIO_DEFINE(ina219_rtio_ctx_##inst, 16, 16);                                 \
                                                                               \
  static struct ina219_data ina219_data_##inst = {0};                          \
  static const struct ina219_config ina219_config_##inst = {                   \
      .bus = {.rtio = {.ctx = &ina219_rtio_ctx_##inst,                         \
                       .iodev = &ina219_iodev_##inst}},                        \
      .current_lsb = DT_INST_PROP(inst, lsb_microamp),                         \
      .r_shunt = DT_INST_PROP(inst, shunt_milliohm),                           \
      .brng = DT_INST_PROP(inst, brng),                                        \
      .pg = DT_INST_PROP(inst, pg),                                            \
      .badc = DT_INST_PROP(inst, badc),                                        \
      .sadc = DT_INST_PROP(inst, sadc),                                        \
      .mode = INA219_MODE_NORMAL};                                             \
                                                                               \
  SENSOR_DEVICE_DT_INST_DEFINE(inst, ina219_init, NULL, &ina219_data_##inst,   \
                               &ina219_config_##inst, POST_KERNEL,             \
                               CONFIG_SENSOR_INIT_PRIORITY, &ina219_api);

DT_INST_FOREACH_STATUS_OKAY(INA219_INIT)
