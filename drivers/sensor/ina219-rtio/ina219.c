#include "ina219.h"
#include "ina219_bus.h"

#include <zephyr/drivers/i2c/rtio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
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
                                          uint16_t *frame_count) {}

static int ina219_decoder_get_size_info(struct sensor_chan_spec chan_spec,
                                        size_t *base_size, size_t *frame_size) {
}

static int ina219_decoder_decode(const uint8_t *buffer,
                                 struct sensor_chan_spec chan_spec,
                                 uint32_t *fit, uint16_t max_count,
                                 void *data_out) {}

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

static void ina219_submit(const struct device *dev,
                          struct rtio_iodev_sqe *iodev_sqe) {}

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
