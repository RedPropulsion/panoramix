#define DT_DRV_COMPAT generic_pwm_servo

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(servo_pwm, CONFIG_SERVO_LOG_LEVEL);

struct servo_pwm_config {
  struct pwm_dt_spec pwm;
  uint32_t min_pulse_ns;   /* pulse width at 0 mdeg, nanoseconds */
  uint32_t max_pulse_ns;   /* pulse width at max_angle_mdeg, nanoseconds */
  uint32_t max_angle_mdeg; /* full-range angle in milli-degrees */
};

static int servo_pwm_set_position(const struct device *dev,
                                  int32_t angle_mdeg) {
  const struct servo_pwm_config *cfg = dev->config;
  uint32_t pulse_ns;
  int ret;

  if (angle_mdeg < 0 || (uint32_t)angle_mdeg > cfg->max_angle_mdeg) {
    LOG_ERR("Angle %d mdeg out of range [0, %u]", angle_mdeg,
            cfg->max_angle_mdeg);
    return -EINVAL;
  }

  /* Linear interpolation: pulse = min + (angle / max_angle) * (max - min) */
  pulse_ns =
      cfg->min_pulse_ns + (uint32_t)(((uint64_t)angle_mdeg *
                                      (cfg->max_pulse_ns - cfg->min_pulse_ns)) /
                                     cfg->max_angle_mdeg);

  ret = pwm_set_pulse_dt(&cfg->pwm, pulse_ns);
  if (ret < 0) {
    LOG_ERR("Failed to set PWM pulse: %d", ret);
    return ret;
  }

  LOG_DBG("position %d mdeg -> pulse %u ns", angle_mdeg, pulse_ns);
  return 0;
}

static int servo_pwm_get_position(const struct device *dev,
                                  int32_t *angle_mdeg) {
  /*
   * PWM servos have no feedback wire.
   */
  return -ENOTSUP;
}

static int servo_pwm_init(const struct device *dev) {
  const struct servo_pwm_config *cfg = dev->config;

  if (!pwm_is_ready_dt(&cfg->pwm)) {
    LOG_ERR("PWM device %s is not ready", cfg->pwm.dev->name);
    return -ENODEV;
  }

  LOG_DBG("ready: min=%u ns, max=%u ns, range=%u mdeg", cfg->min_pulse_ns,
          cfg->max_pulse_ns, cfg->max_angle_mdeg);
  return 0;
}

static DEVICE_API(servo, servo_pwm_api) = {
    .set_position = servo_pwm_set_position,
    .get_position = servo_pwm_get_position,
};

#define SERVO_PWM_INIT(inst)                                                   \
  static const struct servo_pwm_config servo_pwm_cfg_##inst = {                \
      .pwm = PWM_DT_SPEC_INST_GET(inst),                                       \
      .min_pulse_ns = DT_INST_PROP(inst, min_pulse),                           \
      .max_pulse_ns = DT_INST_PROP(inst, max_pulse),                           \
      .max_angle_mdeg = DT_INST_PROP(inst, max_angle),                         \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(inst, servo_pwm_init, NULL, NULL,                      \
                        &servo_pwm_cfg_##inst, POST_KERNEL,                    \
                        CONFIG_SERVO_INIT_PRIORITY, &servo_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(SERVO_PWM_INIT)
