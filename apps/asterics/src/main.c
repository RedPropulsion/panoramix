#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

const struct gpio_dt_spec spec = GPIO_DT_SPEC_GET(DT_NODELABEL(my_led), gpios);

int main(void) {
  if (!gpio_is_ready_dt(&spec)) {
    return 1;
  }

  gpio_pin_configure_dt(&spec, GPIO_OUTPUT_HIGH);

  int x = 0;

  while (1) {
    printk("HEY %d!\n", x++);
    gpio_pin_toggle_dt(&spec);
    k_sleep(K_MSEC(1000));
  }
}
