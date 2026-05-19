#include "zephyr/drivers/pwm.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/regulator.h> // per il controllo alimentazione
#include "runcam.h"

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

static runcam_ctx_t camera_ctx;

int main(void) {
  // const struct device *main_servo = DEVICE_DT_GET(DT_NODELABEL(servo_drogue_pwm));
  /* 1. Recupero il regolatore tramite il suo Label nel DTS */
  const struct device *pwr_dev = DEVICE_DT_GET(DT_NODELABEL(runcam_vdd));

  if (!device_is_ready(pwr_dev))
  {
    LOG_ERR("Regolatore RunCam non pronto!");
    return -ENODEV;
  }

  /* Accensione camera */
  regulator_enable(pwr_dev);
  k_msleep(RUNCAM_BOOT_DELAY_MS);

  /* 2. Recupero la UART dove è definita la RunCam */
  /* DT_BUS(DT_NODELABEL(runcam_device)) risolve automaticamente in &uart5 */
  const struct device *uart_dev = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(runcam_device)));

  if (!device_is_ready(uart_dev))
  {
    LOG_ERR("UART della RunCam non pronta!");
    return -ENODEV;
  }

  /* 3. Inizializzazione Driver */
  int ret = runcam_init(&camera_ctx, uart_dev);
  if (ret < 0)
  {
    LOG_ERR("Fallito init protocollo RunCam: %d", ret);
    return ret;
  }

  /* Richiesta info camera per capire se supporta START/STOP_RECORDING */
  runcam_get_device_info(&camera_ctx);

  LOG_INF("Sistema RunCam pronto su UART5");

  while (1)
  {
    /* Esempio: Registra per 10 secondi e poi ferma */
    LOG_INF("Avvio registrazione...");
    runcam_start_recording(&camera_ctx);

    k_sleep(K_SECONDS(10));

    LOG_INF("Stop registrazione.");
    runcam_stop_recording(&camera_ctx);

    k_sleep(K_SECONDS(5));
    /*
    LOG_INF("VADO A 0");
    servo_set_position(main_servo, 0);
    k_sleep(K_MSEC(5000));

    LOG_INF("VADO A mid");
    servo_set_position(main_servo, 135 * 1000);
    k_sleep(K_MSEC(5000));

    LOG_INF("VADO A MAX");
    servo_set_position(main_servo, 270 * 1000);
    k_sleep(K_MSEC(10000)); */
  }
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf)
{
  const struct gpio_dt_spec error_led =
      GPIO_DT_SPEC_GET(DT_NODELABEL(led_mcu_1), gpios);

  LOG_PANIC();

  while (1)
  {
    LOG_ERR("I'M PANICKING");
    gpio_pin_toggle_dt(&error_led);
    k_busy_wait(500 * 1000);
  }

  k_fatal_halt(reason);
};
