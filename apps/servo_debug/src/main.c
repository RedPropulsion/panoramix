#include "zephyr/drivers/servo.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DOUBLE_CLICK_MS 400

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct gpio_dt_spec button_up =
    GPIO_DT_SPEC_GET(DT_NODELABEL(button_up), gpios);

static const struct gpio_dt_spec button_down =
    GPIO_DT_SPEC_GET(DT_NODELABEL(button_down), gpios);

static const struct device *servo1 = DEVICE_DT_GET(DT_NODELABEL(servo_ch1));
static const struct device *servo2 = DEVICE_DT_GET(DT_NODELABEL(servo_ch2));
static const struct device *servo3 = DEVICE_DT_GET(DT_NODELABEL(servo_ch3));
static const struct device *servo4 = DEVICE_DT_GET(DT_NODELABEL(servo_ch4));

static struct k_work_delayable single_press_work;
static uint32_t last_pressed = 0;
static uint32_t servo_deg_millideg = 270000;

static void single_press_handler(struct k_work *work) {
  if (servo_deg_millideg == 0) {
    return;
  }
  servo_deg_millideg -= 10000;
  LOG_INF("PRESSED UP! Going to %d", servo_deg_millideg);
  servo_set_position(servo1, servo_deg_millideg);
  servo_set_position(servo2, servo_deg_millideg);
  servo_set_position(servo3, servo_deg_millideg);
  servo_set_position(servo4, servo_deg_millideg);
}

void button_handler(const struct device *dev, struct gpio_callback *callback,
                    uint32_t pins) {
  if (pins & BIT(button_up.pin)) {
    uint32_t now = k_uptime_get_32();
    if (now - last_pressed < DOUBLE_CLICK_MS) {
      // Second press arrived in time — cancel the pending single press log
      k_work_cancel_delayable(&single_press_work);
      LOG_INF("DOUBLE CLICK UP! Going to 0");
      servo_deg_millideg = 0;
      servo_set_position(servo1, 0);
      servo_set_position(servo2, 0);
      servo_set_position(servo3, 0);
      servo_set_position(servo4, 0);
      last_pressed = 0; // Reset so a 3rd press doesn't re-trigger double
    } else {
      // First press — wait to see if a second one follows
      k_work_reschedule(&single_press_work, K_MSEC(DOUBLE_CLICK_MS));
      last_pressed = now;
    }
  } else if (pins & BIT(button_down.pin)) {
    if (servo_deg_millideg == 270000) {
      return;
    }
    servo_deg_millideg += 10000;
    LOG_INF("PRESSED DOWN! Going to %d", servo_deg_millideg);
    servo_set_position(servo1, servo_deg_millideg);
    servo_set_position(servo2, servo_deg_millideg);
    servo_set_position(servo3, servo_deg_millideg);
    servo_set_position(servo4, servo_deg_millideg);
  }
}

int main(void) {
  static struct gpio_callback button_up_callback;
  static struct gpio_callback button_down_callback;

  k_work_init_delayable(&single_press_work, single_press_handler);

  gpio_pin_configure_dt(&button_up, GPIO_INPUT);
  gpio_pin_configure_dt(&button_down, GPIO_INPUT);

  gpio_pin_interrupt_configure_dt(&button_up, GPIO_INT_EDGE_FALLING);
  gpio_pin_interrupt_configure_dt(&button_down, GPIO_INT_EDGE_FALLING);

  gpio_init_callback(&button_up_callback, button_handler, BIT(button_up.pin));
  gpio_init_callback(&button_down_callback, button_handler,
                     BIT(button_down.pin));

  gpio_add_callback(button_up.port, &button_up_callback);
  gpio_add_callback(button_down.port, &button_down_callback);

  while (1) {
    k_sleep(K_MSEC(5000));
  }

  return 0;
}
