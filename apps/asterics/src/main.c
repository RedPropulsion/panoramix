#include <mavwrap.h>
#include <string.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sensing/sensing.h>
#include <zephyr/smf.h>

#include <stddef.h>

LOG_MODULE_REGISTER(main);

#define RX_QUEUE_SIZE 16

static const struct device *mavlink_usart =
    DEVICE_DT_GET(DT_NODELABEL(mavlink_usart));

K_MSGQ_DEFINE(rx_queue, sizeof(mavlink_message_t), RX_QUEUE_SIZE,
              sizeof(void *));

const struct device *servos[] = {
    DEVICE_DT_GET(DT_NODELABEL(servo_main_pwm)),
    DEVICE_DT_GET(DT_NODELABEL(servo_drogue_pwm)),
};

struct exec_ctx {
  // MUST be the first element in the struct
  struct smf_ctx ctx;
} exec_ctx_obj;

enum state { BOOT, IDLE, CALIBRATION, MANUAL, STREAM, ARMED, LAUNCH };
// Forward declaration
const struct smf_state states[];

static void set_servo(uint8_t servo_id, uint32_t deg) {
  if (servo_id >= ARRAY_SIZE(servos)) {
    LOG_ERR("Invalid servo_id: %d", servo_id);
    return;
  }

  servo_set_position(servos[servo_id], deg * 1000);
}

static enum smf_state_result boot_run(void *o) {
  LOG_WRN("Not implemented");

  smf_set_state(SMF_CTX(&exec_ctx_obj), &states[IDLE]);
  return SMF_EVENT_HANDLED;
}

static enum smf_state_result idle_run(void *o) {
  mavlink_message_t message;
  int ret = k_msgq_get(&rx_queue, &message, K_NO_WAIT);
  if (ret < 0) {
    return SMF_EVENT_HANDLED;
  }

  LOG_INF("Got message id: %d", message.msgid);

  if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&message, &cmd);

    if (cmd.command == MAV_CMD_DO_SET_MODE) {
      switch ((int)cmd.param2) {
      case 12:
        LOG_INF("GOING TO MANUAL MODE");
        smf_set_state(SMF_CTX(&exec_ctx_obj), &states[MANUAL]);
        break;
      default:
        LOG_ERR("Invalid custom mode code %d", (int)cmd.param2);
        break;
      }
    } else {
      LOG_ERR("Invalid cmd %d", cmd.command);
    }
  }

  return SMF_EVENT_HANDLED;
}

static enum smf_state_result manual_run(void *o) {
  mavlink_message_t message;
  int ret = k_msgq_get(&rx_queue, &message, K_NO_WAIT);
  if (ret < 0) {
    return SMF_EVENT_HANDLED;
  }

  LOG_INF("Got message id: %d", message.msgid);

  if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&message, &cmd);

    if (cmd.command == MAV_CMD_DO_SET_MODE) {
      switch ((int)cmd.param2) {
      case 14:
        LOG_INF("GOING TO IDLE MODE");
        smf_set_state(SMF_CTX(&exec_ctx_obj), &states[MANUAL]);
        break;
      default:
        LOG_ERR("Invalid custom mode code %d", (int)cmd.param2);
        break;
      }
    } else if (cmd.command == MAV_CMD_DO_SET_SERVO) {
      set_servo(cmd.param1, cmd.param2);
    } else {
      LOG_ERR("Invalid cmd %d", cmd.command);
    }
  }

  return SMF_EVENT_HANDLED;
}

const struct smf_state states[] = {
    [BOOT] = SMF_CREATE_STATE(NULL, boot_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(NULL, idle_run, NULL, NULL, NULL),
    [CALIBRATION] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [MANUAL] = SMF_CREATE_STATE(NULL, manual_run, NULL, NULL, NULL),
    [STREAM] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [ARMED] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [LAUNCH] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
};

static void usart_rx_callback(const struct device *dev,
                              const mavlink_message_t *msg, void *user_data) {
  int ret = k_msgq_put(&rx_queue, msg, K_NO_WAIT);
  if (ret < 0) {
    LOG_ERR("Queue overflow, clearing...");
    k_msgq_purge(&rx_queue);
    k_msgq_put(&rx_queue, msg, K_NO_WAIT);
  }
}

int main(void) {
  LOG_INF("The board started!");
  smf_set_initial(SMF_CTX(&exec_ctx_obj), &states[BOOT]);
  mavwrap_start(mavlink_usart, usart_rx_callback, NULL);

  while (1) {
    int32_t ret = smf_run_state(SMF_CTX(&exec_ctx_obj));
    if (ret < 0) {
      LOG_ERR("The state machine crashed: %s\n", strerror(-ret));
    }
  }
  return 0;
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
