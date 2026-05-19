#include <mavwrap.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sensing/sensing.h>

#include <stddef.h>
#include <string.h>

LOG_MODULE_REGISTER(main);

static const struct device *mavlink_usart =
    DEVICE_DT_GET(DT_NODELABEL(mavlink_usart));
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
    LOG_INF("temp=%" PRIsensor_q31_data "\n",
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

const struct device *cacofonics_usart = DEVICE_DT_GET(DT_NODELABEL(usart3));

static uint8_t rx_buffer1[256] = {0};
static uint8_t rx_buffer2[256] = {0};
static uint8_t *active_buf = rx_buffer1;
static uint8_t *next_buf = NULL;

// void rx_callback(const struct device *dev, struct uart_event *evt,
//                  void *user_data) {
//
//   uint8_t *data;
//   int err;
//
//   switch (evt->type) {
//
//   case UART_RX_RDY:
//     data = evt->data.rx.buf + evt->data.rx.offset;
//     size_t len = evt->data.rx.len;
//     if (len > 0) {
//       LOG_INF("RX %d bytes: %.*s", len, len, data);
//     }
//     break;
//
//   case UART_RX_BUF_REQUEST:
//     if (next_buf == NULL) {
//       next_buf = rx_buffer2;
//       err = uart_rx_buf_rsp(dev, next_buf, sizeof(rx_buffer2));
//       if (err < 0) {
//         LOG_ERR("Failed to provide second buffer: %s", strerror(-err));
//       }
//     }
//     break;
//
//   case UART_RX_BUF_RELEASED:
//     if (evt->data.rx_buf.buf == rx_buffer1) {
//       active_buf = rx_buffer2;
//     } else if (evt->data.rx_buf.buf == rx_buffer2) {
//       active_buf = rx_buffer1;
//     }
//     next_buf = NULL;
//     break;
//
//   case UART_RX_DISABLED:
//     next_buf = NULL;
//     active_buf = rx_buffer1;
//     err = uart_rx_enable(dev, rx_buffer1, sizeof(rx_buffer1) - 1,
//                          10 * USEC_PER_MSEC);
//     if (err < 0) {
//       LOG_ERR("Failed to re-enable RX: %s", strerror(-err));
//     }
//     break;
//
//   case UART_RX_STOPPED:
//     LOG_WRN("RX stopped, reason: %d", evt->data.rx_stop.reason);
//     break;
//
//   case UART_TX_DONE:
//     LOG_INF("Message sent!");
//     break;
//
//   default:
//     break;
//   }
// }

static void usart_rx_callback(const struct device *dev,
                              const mavlink_message_t *msg, void *user_data) {
  mavlink_ping_t ping;

  LOG_INF("USART: Got msgid: %d", msg->msgid);

  mavlink_msg_ping_decode(msg, &ping);

  LOG_INF("time_usec: %d", ping.time_usec);
}

int main(void) {
  LOG_INF("The board started!");
  const uint8_t message[] = "Hi CacofonICS!";

  mavwrap_start(mavlink_usart, usart_rx_callback, NULL);

  // int err = uart_callback_set(cacofonics_usart, rx_callback, NULL);
  // if (err < 0) {
  //   LOG_ERR("Unable to set rx callback for cacofonics usart: %s",
  //           strerror(-err));
  //   return 1;
  // }
  //
  // err = uart_rx_enable(cacofonics_usart, rx_buffer1, sizeof(rx_buffer1) - 1,
  //                      10 * USEC_PER_MSEC);
  // if (err < 0) {
  //   LOG_ERR("Unable to enable rx for cacofonics usart: %s", strerror(-err));
  //   return 1;
  // }

  while (1) {
    k_sleep(K_MSEC(5000));
    mavlink_message_t msg;
    mavlink_msg_ping_pack(1, 1, &msg, 69420, 1, 4, 1);

    // int err = uart_tx(cacofonics_usart, message, sizeof(message), 10 * 1000);
    int err = mavwrap_send_message(mavlink_usart, &msg);
    if (err < 0) {
      LOG_ERR("Unable to send data: %s", strerror(-err));
      return 1;
    }
  }
  return 0;

  // k_sleep(K_MSEC(500));
  // const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(mcu_ms5611));

  // const struct device *gyro_prova =
  //     DEVICE_DT_GET(DT_NODELABEL(mcu_bmi088_gyro));

  // struct sensor_value gyro_data[3]; // ARRAY di 3 elementi: 0=X, 1=Y, 2=Z

  // while (!device_is_ready(dev)) {
  //   LOG_INF("Waiting for %s to be ready...", dev->name);
  //   k_sleep(K_MSEC(500));
  // }

  // while (!device_is_ready(gyro_prova)) {
  //   LOG_INF("Waiting for %s to be ready...", gyro_prova->name);
  //   k_sleep(K_MSEC(500));
  // }

  // LOG_INF("Dispositivo %s trovato e pronto!", gyro_prova->name);

  // for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
  //   gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_LOW);
  // }
  // int ret;
  // while (1) {
  //   // 4. FETCH: Chiediamo al driver di leggere i registri hardware
  //   ret = sensor_sample_fetch(gyro_prova);
  //   if (ret < 0) {
  //     LOG_ERR("Fetch fallito: %d", ret);
  //     continue; // Salta al prossimo giro
  //   }

  //   // 5. GET: Copiamo i dati nella nostra struttura
  //   ret = sensor_channel_get(gyro_prova, SENSOR_CHAN_GYRO_XYZ, gyro_data);
  //   if (ret < 0) {
  //     LOG_ERR("Get fallito: %d", ret);
  //   } else {
  //     // 6. Stampa formattata
  //     // Usiamo il formato intero.decimale per evitare problemi con float se
  //     non
  //     // configurati
  //     LOG_INF("GYRO (rad/s): X=%d.%06d | Y=%d.%06d | Z=%d.%06d",
  //             gyro_data[0].val1, abs(gyro_data[0].val2), gyro_data[1].val1,
  //             abs(gyro_data[1].val2), gyro_data[2].val1,
  //             abs(gyro_data[2].val2));

  //     // Opzionale: Converti in gradi/secondo (1 rad = ~57.295 gradi)
  //     // (Richiede che tu faccia i calcoli a mano o abiliti i float)
  //   }

  //   // Leggiamo a 10Hz (ogni 100ms)
  //   k_sleep(K_MSEC(100));
  // }

  // while (1) {
  //   for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
  //     LOG_INF("Setting led %d!", i + 1);
  //     gpio_pin_set_dt(&leds[i], 1);

  //     k_sleep(K_MSEC(1000));
  //   }

  //   k_sleep(K_MSEC(1000));

  //   for (size_t i = 0; i < (sizeof leds) / (sizeof leds[0]); i++) {
  //     LOG_INF("Resetting led %d!", i + 1);
  //     gpio_pin_set_dt(&leds[i], 0);

  //     k_sleep(K_MSEC(1000));
  //   }

  //   while (1) {
  //     int ret = sensor_sample_fetch(dev);
  //     if (ret < 0) {
  //       LOG_ERR("Cannot retreive sample: %d", ret);
  //       return 1;
  //     }

  //     struct sensor_value val = {0};
  //     ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &val);
  //     if (ret < 0) {
  //       LOG_ERR("Cannot retreive sample: %d", ret);
  //       return 1;
  //     }

  //     LOG_INF("Read temperature %d.%d", val.val1, val.val2);

  //     k_sleep(K_MSEC(500));
  //   }

  //   // 1. Dichiara un ARRAY di 3 elementi (uno per asse: 0=X, 1=Y, 2=Z)
  //   struct sensor_value val_gyr[3];

  //   // 2. IMPORTANTE: Prima devi fare il fetch dei dati dal sensore
  //   ret = sensor_sample_fetch(gyro_prova);
  //   if (ret < 0) {
  //     LOG_ERR("Errore nel fetch del campione: %d", ret);
  //     return 1; // O gestisci l'errore
  //   }

  //   // 3. Ottieni i dati sul canale XYZ passando l'array
  //   ret = sensor_channel_get(gyro_prova, SENSOR_CHAN_GYRO_XYZ, val_gyr);
  //   if (ret < 0) {
  //     LOG_ERR("Impossibile recuperare i dati: %d", ret);
  //   } else {
  //     // 4. Stampa i dati. Nota che ogni asse ha la sua parte intera (val1) e
  //     // frazionaria (val2). Qui stampiamo solo la parte intera (val1) per
  //     // semplicità, come nel tuo esempio originale.
  //     LOG_INF("Read gyro x=%d, y=%d, z=%d",
  //             val_gyr[0].val1,  // Asse X
  //             val_gyr[1].val1,  // Asse Y
  //             val_gyr[2].val1); // Asse Z
  //   }

  //   /*
  //       int ret = sensor_sample_fetch(dev);
  //       if (ret < 0) {
  //         LOG_ERR("Cannot retreive sample: %d", ret);
  //       } */

  //   /* k_panic(); */
  // }
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
