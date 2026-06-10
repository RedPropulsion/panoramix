#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sensing/sensing.h>

#include <stddef.h>

static const char *const sensor_channel_name[SENSOR_CHAN_COMMON_COUNT] = {
    [SENSOR_CHAN_ACCEL_X] = "accel_x",
    [SENSOR_CHAN_ACCEL_Y] = "accel_y",
    [SENSOR_CHAN_ACCEL_Z] = "accel_z",
    [SENSOR_CHAN_ACCEL_XYZ] = "accel_xyz",
    [SENSOR_CHAN_GYRO_X] = "gyro_x",
    [SENSOR_CHAN_GYRO_Y] = "gyro_y",
    [SENSOR_CHAN_GYRO_Z] = "gyro_z",
    [SENSOR_CHAN_GYRO_XYZ] = "gyro_xyz",
    [SENSOR_CHAN_MAGN_X] = "magn_x",
    [SENSOR_CHAN_MAGN_Y] = "magn_y",
    [SENSOR_CHAN_MAGN_Z] = "magn_z",
    [SENSOR_CHAN_MAGN_XYZ] = "magn_xyz",
    [SENSOR_CHAN_DIE_TEMP] = "die_temp",
    [SENSOR_CHAN_AMBIENT_TEMP] = "ambient_temp",
    [SENSOR_CHAN_PRESS] = "press",
    [SENSOR_CHAN_PROX] = "prox",
    [SENSOR_CHAN_HUMIDITY] = "humidity",
    [SENSOR_CHAN_AMBIENT_LIGHT] = "ambient_light",
    [SENSOR_CHAN_LIGHT] = "light",
    [SENSOR_CHAN_IR] = "ir",
    [SENSOR_CHAN_RED] = "red",
    [SENSOR_CHAN_GREEN] = "green",
    [SENSOR_CHAN_BLUE] = "blue",
    [SENSOR_CHAN_ALTITUDE] = "altitude",
    [SENSOR_CHAN_PM_1_0_CF] = "pm_1_0_cf",
    [SENSOR_CHAN_PM_2_5_CF] = "pm_2_5_cf",
    [SENSOR_CHAN_PM_10_CF] = "pm_10_cf",
    [SENSOR_CHAN_PM_1_0] = "pm_1_0",
    [SENSOR_CHAN_PM_2_5] = "pm_2_5",
    [SENSOR_CHAN_PM_10] = "pm_10",
    [SENSOR_CHAN_PM_0_3_COUNT] = "pm_0_3_count",
    [SENSOR_CHAN_PM_0_5_COUNT] = "pm_0_5_count",
    [SENSOR_CHAN_PM_1_0_COUNT] = "pm_1_0_count",
    [SENSOR_CHAN_PM_2_5_COUNT] = "pm_2_5_count",
    [SENSOR_CHAN_PM_5_COUNT] = "pm_5_0_count",
    [SENSOR_CHAN_PM_10_COUNT] = "pm_10_count",
    [SENSOR_CHAN_DISTANCE] = "distance",
    [SENSOR_CHAN_CO2] = "co2",
    [SENSOR_CHAN_O2] = "o2",
    [SENSOR_CHAN_VOC] = "voc",
    [SENSOR_CHAN_GAS_RES] = "gas_resistance",
    [SENSOR_CHAN_FLOW_RATE] = "flow_rate",
    [SENSOR_CHAN_VOLTAGE] = "voltage",
    [SENSOR_CHAN_VSHUNT] = "vshunt",
    [SENSOR_CHAN_CURRENT] = "current",
    [SENSOR_CHAN_POWER] = "power",
    [SENSOR_CHAN_RESISTANCE] = "resistance",
    [SENSOR_CHAN_ROTATION] = "rotation",
    [SENSOR_CHAN_POS_DX] = "pos_dx",
    [SENSOR_CHAN_POS_DY] = "pos_dy",
    [SENSOR_CHAN_POS_DZ] = "pos_dz",
    [SENSOR_CHAN_POS_DXYZ] = "pos_dxyz",
    [SENSOR_CHAN_RPM] = "rpm",
    [SENSOR_CHAN_FREQUENCY] = "frequency",
    [SENSOR_CHAN_GAUGE_VOLTAGE] = "gauge_voltage",
    [SENSOR_CHAN_GAUGE_AVG_CURRENT] = "gauge_avg_current",
    [SENSOR_CHAN_GAUGE_STDBY_CURRENT] = "gauge_stdby_current",
    [SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT] = "gauge_max_load_current",
    [SENSOR_CHAN_GAUGE_TEMP] = "gauge_temp",
    [SENSOR_CHAN_GAUGE_STATE_OF_CHARGE] = "gauge_state_of_charge",
    [SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY] = "gauge_full_cap",
    [SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY] = "gauge_remaining_cap",
    [SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY] = "gauge_nominal_cap",
    [SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY] = "gauge_full_avail_cap",
    [SENSOR_CHAN_GAUGE_AVG_POWER] = "gauge_avg_power",
    [SENSOR_CHAN_GAUGE_STATE_OF_HEALTH] = "gauge_state_of_health",
    [SENSOR_CHAN_GAUGE_TIME_TO_EMPTY] = "gauge_time_to_empty",
    [SENSOR_CHAN_GAUGE_TIME_TO_FULL] = "gauge_time_to_full",
    [SENSOR_CHAN_GAUGE_CYCLE_COUNT] = "gauge_cycle_count",
    [SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE] = "gauge_design_voltage",
    [SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE] = "gauge_desired_voltage",
    [SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT] =
        "gauge_desired_charging_current",
    [SENSOR_CHAN_GAME_ROTATION_VECTOR] = "game_rotation_vector",
    [SENSOR_CHAN_GRAVITY_VECTOR] = "gravity_vector",
    [SENSOR_CHAN_GBIAS_XYZ] = "gbias_xyz",
    [SENSOR_CHAN_ENCODER_COUNT] = "encoder_count",
    [SENSOR_CHAN_ALL] = "all",
};

LOG_MODULE_REGISTER(main);

SENSOR_DT_READ_IODEV(mcu_ms5611_iodev, DT_NODELABEL(mcu_ms5611),
                     {
                         SENSOR_CHAN_PRESS,
                         0,
                     },
                     {SENSOR_CHAN_AMBIENT_TEMP, 0});

SENSOR_DT_READ_IODEV(mcu_ina219_iodev, DT_NODELABEL(ina219_mcu),
                     {
                         SENSOR_CHAN_VOLTAGE,
                         0,
                     });

RTIO_DEFINE_WITH_MEMPOOL(sensor_ctx, 16, 16, 16, 256, sizeof(void *));

static void on_sensor_data(int ret, uint8_t *buf, uint32_t buf_len,
                           void *userdata) {
  const struct rtio_iodev *iodev_sqe = userdata;
  const struct sensor_read_config *cfg = iodev_sqe->data;
  const struct device *dev = cfg->sensor;

  if (ret < 0) {
    LOG_ERR("Reading failed for %s: %s", dev->name, strerror(-ret));
    return;
  }

  const struct sensor_decoder_api *decoder;
  ret = sensor_get_decoder(dev, &decoder);
  if (ret < 0) {
    LOG_ERR("Couldn't get decoder for %s: %s", dev->name, strerror(-ret));
    return;
  }

  for (size_t i = 0; i < cfg->count; i++) {
    uint16_t frame_count;
    if (decoder->get_frame_count(buf, cfg->channels[i], &frame_count) != 0)
      continue;
    uint32_t fit = 0;
    struct sensor_q31_data out;
    decoder->decode(buf, cfg->channels[i], &fit, 1, &out);
    if (fit > 0) {
      LOG_INF("%s=%" PRIsensor_q31_data,
              sensor_channel_name[cfg->channels[i].chan_type],
              PRIsensor_q31_data_arg(out, 0));
    }
  }
}

static void sensor_processing_thread(void *a, void *b, void *c) {
  while (1) {
    sensor_processing_with_callback(&sensor_ctx, on_sensor_data);
  }
}
K_THREAD_DEFINE(sensor_proc_tid, 2048, sensor_processing_thread, NULL, NULL,
                NULL, 5, 0, 0);

int main(void) {
  const struct device *main_servo =
      DEVICE_DT_GET(DT_NODELABEL(servo_drogue_pwm));

  // while (1) {
  //   LOG_INF("VADO A 0");
  //   servo_set_position(main_servo, 0);
  //   k_sleep(K_MSEC(5000));
  //
  //   LOG_INF("VADO A mid");
  //   servo_set_position(main_servo, 135 * 1000);
  //   k_sleep(K_MSEC(5000));
  //
  //   LOG_INF("VADO A MAX");
  //   servo_set_position(main_servo, 270 * 1000);
  //   k_sleep(K_MSEC(10000));
  // }
  while (1) {
    int ret = sensor_read_async_mempool(&mcu_ms5611_iodev, &sensor_ctx,
                                        &mcu_ms5611_iodev);
    if (ret < 0) {
      LOG_ERR("Couldn't perform read: %s", strerror(-ret));
    }

    ret = sensor_read_async_mempool(&mcu_ina219_iodev, &sensor_ctx,
                                    &mcu_ina219_iodev);
    if (ret < 0) {
      LOG_ERR("Couldn't perform read: %s", strerror(-ret));
    }

    k_sleep(K_MSEC(5000));
  }
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf) {
  const struct gpio_dt_spec error_led =
      GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_1), gpios);

  LOG_PANIC();

  while (1) {
    gpio_pin_toggle_dt(&error_led);
    k_busy_wait(500 * 1000);
  }

  k_fatal_halt(reason);
};
