#include <zephyr/drivers/gpio.h>
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

int main(void) {
  /* if (!gpio_is_ready_dt(&spec)) { */
  /*   return 1; */
  /* } */

  for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
    gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_LOW);
  }

  while (1) {
    for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
      LOG_INF("Setting led %d!", i + 1);
      gpio_pin_set_dt(&leds[i], 1);

      k_sleep(K_MSEC(1000));
    }

    k_sleep(K_MSEC(1000));

    for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
      LOG_INF("Resetting led %d!", i + 1);
      gpio_pin_set_dt(&leds[i], 0);

      k_sleep(K_MSEC(1000));
    }

    k_panic();
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
