#include "zephyr/drivers/pwm.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <stddef.h>

LOG_MODULE_REGISTER(main);

const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_3), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_4), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_5), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_6), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_7), gpios),
};
struct servo_pwm_config {
  struct pwm_dt_spec pwm;
};

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
};
