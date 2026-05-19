#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/util.h>

#include <mavwrap.h>

#include <string.h>

#define MAVLINK_LORA_LABEL DT_NODELABEL(mavlink_lora)
#define MAVLINK_USART_LABEL DT_NODELABEL(mavlink_usart)

#define LORA_SNR_LOW -50
#define LORA_SNR_HIGH -15

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct device *mavlink_lora = DEVICE_DT_GET(MAVLINK_LORA_LABEL);
static const struct device *mavlink_usart = DEVICE_DT_GET(MAVLINK_USART_LABEL);

static const struct gpio_dt_spec lora_leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(lora_led_1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(lora_led_2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(lora_led_3), gpios),
};

static int lora_set_state_led(int8_t snr) {
  // Blink leds, then set new value
  for (int i = 0; i < ARRAY_SIZE(lora_leds); i++) {
    gpio_pin_set_dt(&lora_leds[i], 0);
  }
  k_sleep(K_MSEC(200));

  int level = 0;
  if (snr < LORA_SNR_LOW) {
    level = 1;
  } else if (snr < LORA_SNR_HIGH) {
    level = 2;
  } else {
    level = 3;
  }

  for (int i = 0; i < level; i++) {
    gpio_pin_set_dt(&lora_leds[i], 1);
  }

  return 0;
}

static void rx_callback(const struct device *dev, const mavlink_message_t *msg,
                        void *user_data) {
  int from_lora = dev == mavlink_lora;

  LOG_INF("%s: Got msgid: %d", from_lora ? "LORA" : "UART", msg->msgid);

  struct mavwrap_stats stats;
  int ret = mavwrap_get_stats(dev, &stats);
  if (ret < 0) {
    LOG_ERR("\tCouldn't get %s stats: %s", from_lora ? "LORA" : "UART",
            strerror(-ret));
  } else {
    if (from_lora) {
      LOG_INF("\tRSSI: %d\tSNR: %d", stats.rx_rssi, stats.rx_snr);
      lora_set_state_led(stats.rx_snr);
    }
    if (stats.tx_errors) {
      LOG_WRN("\ttx errors: %d", stats.tx_errors);
    }
  }

  mavwrap_send_message(from_lora ? mavlink_usart : mavlink_lora, msg);
}

int main(void) {
  LOG_INF("Booting...");
  k_sleep(K_MSEC(5));
  LOG_INF("Running");

  mavwrap_start(mavlink_lora, rx_callback, NULL);
  mavwrap_start(mavlink_usart, rx_callback, NULL);

  return 0;
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf) {
  const struct gpio_dt_spec error_led =
      GPIO_DT_SPEC_GET(DT_ALIAS(error_led), gpios);

  LOG_PANIC();

  while (1) {
    gpio_pin_toggle_dt(&error_led);
    k_busy_wait(500 * 1000);
  }

  k_fatal_halt(reason);
}
