#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <stddef.h>

LOG_MODULE_REGISTER(main);

// TODO change to sensr gpio
const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_3), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_4), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_5), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_6), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_7), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_8), gpios),
};

int main(void) {
  /* if (!gpio_is_ready_dt(&spec)) { */
  /*   return 1; */
  /* } */


  for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
    gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_LOW);
  }

  for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
      LOG_INF("Setting led %d!", i + 1);
      gpio_pin_set_dt(&leds[i], 1);

      k_sleep(K_MSEC(100));
    }

     k_sleep(K_MSEC(100));
    
    for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
      LOG_INF("Resetting led %d!", i + 1);
      gpio_pin_set_dt(&leds[i], 0);

      k_sleep(K_MSEC(100));
    }
    
    gpio_pin_set_dt(&leds[7], 1);
  k_sleep(K_MSEC(500));
  const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(sensor_h3lis)); 
  LOG_INF("dev pointer: %p", dev);
  LOG_INF("dev name: %s", dev->name);
  LOG_INF("initialized: %d", dev->state->initialized);
  LOG_INF("init_res: %d", dev->state->init_res);
  while (!device_is_ready(dev)) {
    LOG_INF("Waiting for %s to be ready...", dev->name);
    k_sleep(K_MSEC(500));
  }


  while (1) {
    /*for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
      LOG_INF("Setting led %d!", i + 1);
      gpio_pin_set_dt(&leds[i], 1);

      k_sleep(K_MSEC(1000));
    }

    / k_sleep(K_MSEC(1000));
    /
    for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
      LOG_INF("Resetting led %d!", i + 1);
      gpio_pin_set_dt(&leds[i], 0);

      k_sleep(K_MSEC(1000));
    }
   */
  LOG_INF("Setting led %d!", 7);
  gpio_pin_set_dt(&leds[7], 1);

    k_sleep(K_MSEC(1000));

    int ret = sensor_sample_fetch(dev);
    if (ret < 0) {
      LOG_ERR("Cannot retreive sample: %d", ret);
      return 1;
    }

    struct sensor_value val[3];
    ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, val); 
    if (ret < 0) {
      LOG_ERR("Cannot retreive sample: %d", ret);
      return 1;
    }
    LOG_INF("Reading acceleration");
    LOG_INF("X: %d.%06d m/s²", val[0].val1, val[0].val2);
    LOG_INF("Y: %d.%06d m/s²", val[1].val1, val[1].val2);
    LOG_INF("Z: %d.%06d m/s²", val[2].val1, val[2].val2);
    /* k_panic(); */
  }
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf) {
  const struct gpio_dt_spec error_led =
      GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_1), gpios);

  LOG_PANIC();

  while (1) {
    LOG_ERR("I'M PANICKING");
    gpio_pin_toggle_dt(&error_led);
    k_busy_wait(500 * 1000);
  }

  k_fatal_halt(reason);
}
