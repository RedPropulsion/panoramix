#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

const struct device *asterics_usart = DEVICE_DT_GET(DT_NODELABEL(usart1));

static uint8_t rx_buffer1[256] = {0};
static uint8_t rx_buffer2[256] = {0};
static uint8_t *active_buf = rx_buffer1;
static uint8_t *next_buf = NULL;

void rx_callback(const struct device *dev, struct uart_event *evt,
                 void *user_data) {

  uint8_t *data;
  int err;

  switch (evt->type) {

  case UART_RX_RDY:
    data = evt->data.rx.buf + evt->data.rx.offset;
    size_t len = evt->data.rx.len;
    if (len > 0) {
      LOG_INF("RX %d bytes: %.*s", len, len, data);
    }
    break;

  case UART_RX_BUF_REQUEST:
    if (next_buf == NULL) {
      next_buf = rx_buffer2;
      err = uart_rx_buf_rsp(dev, next_buf, sizeof(rx_buffer2));
      if (err < 0) {
        LOG_ERR("Failed to provide second buffer: %s", strerror(-err));
      }
    }
    break;

  case UART_RX_BUF_RELEASED:
    if (evt->data.rx_buf.buf == rx_buffer1) {
      active_buf = rx_buffer2;
    } else if (evt->data.rx_buf.buf == rx_buffer2) {
      active_buf = rx_buffer1;
    }
    next_buf = NULL;
    break;

  case UART_RX_DISABLED:
    next_buf = NULL;
    active_buf = rx_buffer1;
    err = uart_rx_enable(dev, rx_buffer1, sizeof(rx_buffer1) - 1,
                         10 * USEC_PER_MSEC);
    if (err < 0) {
      LOG_ERR("Failed to re-enable RX: %s", strerror(-err));
    }
    break;

  case UART_RX_STOPPED:
    LOG_WRN("RX stopped, reason: %d", evt->data.rx_stop.reason);
    break;

  case UART_TX_DONE:
    LOG_INF("Message sent!");
    break;

  default:
    break;
  }
}

int main(void) {
  LOG_INF("The board started!");
  const uint8_t message[] = "Hi AsterICS!";

  int err = uart_callback_set(asterics_usart, rx_callback, NULL);
  if (err < 0) {
    LOG_ERR("Unable to set rx callback for asterics usart: %s", strerror(-err));
    return 1;
  }
  err = uart_rx_enable(asterics_usart, rx_buffer1, sizeof(rx_buffer1) - 1,
                       10 * USEC_PER_MSEC);
  if (err < 0) {
    LOG_ERR("Unable to enable rx callback for asterics usart: %s",
            strerror(-err));
    return 1;
  }

  while (1) {
    k_sleep(K_MSEC(5000));

    int err = uart_tx(asterics_usart, message, sizeof(message), 10 * 1000);
    if (err < 0) {
      LOG_ERR("Unable to send data: %s", strerror(-err));
      return 1;
    }
  }

  return 0;
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf) {
  LOG_PANIC();

  while (1) {
    LOG_ERR("I'M PANICKING");
    // gpio_pin_toggle_dt(&error_led);
    k_busy_wait(500 * 1000);
  }

  k_fatal_halt(reason);
}
