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

int main(void) {
  /* if (!gpio_is_ready_dt(&spec)) { */
  /*   return 1; */
  /* } */

  k_sleep(K_MSEC(500));
  const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(mcu_ms5611));

  const struct device *gyro_prova =
      DEVICE_DT_GET(DT_NODELABEL(mcu_bmi088_gyro));

  struct sensor_value gyro_data[3]; // ARRAY di 3 elementi: 0=X, 1=Y, 2=Z

  while (!device_is_ready(dev)) {
    LOG_INF("Waiting for %s to be ready...", dev->name);
    k_sleep(K_MSEC(500));
  }

  while (!device_is_ready(gyro_prova)) {
    LOG_INF("Waiting for %s to be ready...", gyro_prova->name);
    k_sleep(K_MSEC(500));
  }

  LOG_INF("Dispositivo %s trovato e pronto!", gyro_prova->name);

  for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
    gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_LOW);
  }
  int ret;
  while (1) {
    // 4. FETCH: Chiediamo al driver di leggere i registri hardware
    ret = sensor_sample_fetch(gyro_prova);
    if (ret < 0) {
      LOG_ERR("Fetch fallito: %d", ret);
      continue; // Salta al prossimo giro
    }

    // 5. GET: Copiamo i dati nella nostra struttura
    ret = sensor_channel_get(gyro_prova, SENSOR_CHAN_GYRO_XYZ, gyro_data);
    if (ret < 0) {
      LOG_ERR("Get fallito: %d", ret);
    } else {
      // 6. Stampa formattata
      // Usiamo il formato intero.decimale per evitare problemi con float se non
      // configurati
      LOG_INF("GYRO (rad/s): X=%d.%06d | Y=%d.%06d | Z=%d.%06d",
              gyro_data[0].val1, abs(gyro_data[0].val2), gyro_data[1].val1,
              abs(gyro_data[1].val2), gyro_data[2].val1,
              abs(gyro_data[2].val2));

      // Opzionale: Converti in gradi/secondo (1 rad = ~57.295 gradi)
      // (Richiede che tu faccia i calcoli a mano o abiliti i float)
    }

    // Leggiamo a 10Hz (ogni 100ms)
    k_sleep(K_MSEC(100));
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

    while (1) {
      int ret = sensor_sample_fetch(dev);
      if (ret < 0) {
        LOG_ERR("Cannot retreive sample: %d", ret);
        return 1;
      }

      struct sensor_value val = {0};
      ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &val);
      if (ret < 0) {
        LOG_ERR("Cannot retreive sample: %d", ret);
        return 1;
      }

      LOG_INF("Read temperature %d.%d", val.val1, val.val2);

      k_sleep(K_MSEC(500));
    }

    // 1. Dichiara un ARRAY di 3 elementi (uno per asse: 0=X, 1=Y, 2=Z)
    struct sensor_value val_gyr[3];

    // 2. IMPORTANTE: Prima devi fare il fetch dei dati dal sensore
    ret = sensor_sample_fetch(gyro_prova);
    if (ret < 0) {
      LOG_ERR("Errore nel fetch del campione: %d", ret);
      return 1; // O gestisci l'errore
    }

    // 3. Ottieni i dati sul canale XYZ passando l'array
    ret = sensor_channel_get(gyro_prova, SENSOR_CHAN_GYRO_XYZ, val_gyr);
    if (ret < 0) {
      LOG_ERR("Impossibile recuperare i dati: %d", ret);
    } else {
      // 4. Stampa i dati. Nota che ogni asse ha la sua parte intera (val1) e
      // frazionaria (val2). Qui stampiamo solo la parte intera (val1) per
      // semplicità, come nel tuo esempio originale.
      LOG_INF("Read gyro x=%d, y=%d, z=%d",
              val_gyr[0].val1,  // Asse X
              val_gyr[1].val1,  // Asse Y
              val_gyr[2].val1); // Asse Z
    }

    /*
        int ret = sensor_sample_fetch(dev);
        if (ret < 0) {
          LOG_ERR("Cannot retreive sample: %d", ret);
        } */

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
};
