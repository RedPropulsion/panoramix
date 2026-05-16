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

LOG_MODULE_REGISTER(main);

const struct device *ms5611 = DEVICE_DT_GET(DT_NODELABEL(mcu_ms5611));
SENSOR_DT_READ_IODEV(mcu_ms5611_iodev, DT_NODELABEL(mcu_ms5611),
                     {
                         SENSOR_CHAN_PRESS,
                         0,
                     },
                     {SENSOR_CHAN_AMBIENT_TEMP, 0});

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

  struct sensor_q31_data sensor_data;

  struct sensor_chan_spec press_ch = {SENSOR_CHAN_PRESS, 0};
  uint32_t fit = 0;

  while (decoder->decode(buf, press_ch, &fit, 1, &sensor_data) > 0) {
    LOG_INF("press=%" PRIsensor_q31_data "\n",
            PRIsensor_q31_data_arg(sensor_data, 0));
  }

  struct sensor_chan_spec temp_ch = {SENSOR_CHAN_AMBIENT_TEMP, 0};
  fit = 0;

  while (decoder->decode(buf, temp_ch, &fit, 1, &sensor_data) > 0) {
    LOG_INF("press=%" PRIsensor_q31_data "\n",
            PRIsensor_q31_data_arg(sensor_data, 0));
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

  while (1) {
    LOG_INF("VADO A 0");
    servo_set_position(main_servo, 0);
    k_sleep(K_MSEC(5000));

    LOG_INF("VADO A mid");
    servo_set_position(main_servo, 135 * 1000);
    k_sleep(K_MSEC(5000));

    LOG_INF("VADO A MAX");
    servo_set_position(main_servo, 270 * 1000);
    k_sleep(K_MSEC(10000));
  }
  while (1) {
    LOG_INF("HEY");
    int ret = sensor_read_async_mempool(&mcu_ms5611_iodev, &sensor_ctx,
                                        &mcu_ms5611_iodev);
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
