#include <stddef.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sensing/sensing.h>
#include <zephyr/smf.h>

#include <mavwrap.h>

LOG_MODULE_REGISTER(main);

#define RX_QUEUE_SIZE 16

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

int main(void) {
  smf_set_initial(SMF_CTX(&exec_ctx_obj), &states[BOOT]);

  while (1) {
    int32_t ret = smf_run_state(SMF_CTX(&exec_ctx_obj));
    if (ret < 0) {
      LOG_ERR("The state machine crashed: %s\n", strerror(-ret));
    }
    // =======
    // const struct device *ms5611 = DEVICE_DT_GET(DT_NODELABEL(mcu_ms5611));
    // SENSOR_DT_READ_IODEV(mcu_ms5611_iodev, DT_NODELABEL(mcu_ms5611),
    //                      {
    //                          SENSOR_CHAN_PRESS,
    //                          0,
    //                      },
    //                      {SENSOR_CHAN_AMBIENT_TEMP, 0});
    //
    // RTIO_DEFINE_WITH_MEMPOOL(sensor_ctx, 16, 16, 16, 256, sizeof(void *));
    //
    // static void on_sensor_data(int ret, uint8_t *buf, uint32_t buf_len,
    //                            void *userdata) {
    //   const struct rtio_iodev *iodev_sqe = userdata;
    //   const struct sensor_read_config *cfg = iodev_sqe->data;
    //   const struct device *dev = cfg->sensor;
    //
    //   if (ret < 0) {
    //     LOG_ERR("Reading failed for %s: %s", dev->name, strerror(-ret));
    //     return;
    //   }
    //
    //   const struct sensor_decoder_api *decoder;
    //   ret = sensor_get_decoder(dev, &decoder);
    //   if (ret < 0) {
    //     LOG_ERR("Couldn't get decoder for %s: %s", dev->name,
    //     strerror(-ret)); return;
    //   }
    //
    //   struct sensor_q31_data sensor_data;
    //
    //   struct sensor_chan_spec press_ch = {SENSOR_CHAN_PRESS, 0};
    //   uint32_t fit = 0;
    //
    //   while (decoder->decode(buf, press_ch, &fit, 1, &sensor_data) > 0) {
    //     LOG_INF("press=%" PRIsensor_q31_data "\n",
    //             PRIsensor_q31_data_arg(sensor_data, 0));
    //   }
    //
    //   struct sensor_chan_spec temp_ch = {SENSOR_CHAN_AMBIENT_TEMP, 0};
    //   fit = 0;
    //
    //   while (decoder->decode(buf, temp_ch, &fit, 1, &sensor_data) > 0) {
    //     LOG_INF("temp=%" PRIsensor_q31_data "\n",
    //             PRIsensor_q31_data_arg(sensor_data, 0));
    //   }
    // }
    //
    // static void sensor_processing_thread(void *a, void *b, void *c) {
    //   while (1) {
    //     sensor_processing_with_callback(&sensor_ctx, on_sensor_data);
    //   }
    // }
    // K_THREAD_DEFINE(sensor_proc_tid, 2048, sensor_processing_thread, NULL,
    // NULL,
    //                 NULL, 5, 0, 0);
    //
    // int main(void) {
    //   const struct device *main_servo =
    //       DEVICE_DT_GET(DT_NODELABEL(servo_drogue_pwm));
    //
    //   while (1) {
    //     LOG_INF("VADO A 0");
    //     servo_set_position(main_servo, 0);
    //     k_sleep(K_MSEC(5000));
    //
    //     LOG_INF("VADO A mid");
    //     servo_set_position(main_servo, 135 * 1000);
    //     k_sleep(K_MSEC(5000));
    //
    //     LOG_INF("VADO A MAX");
    //     servo_set_position(main_servo, 270 * 1000);
    //     k_sleep(K_MSEC(10000));
    //   }
    //   while (1) {
    //     int ret = sensor_read_async_mempool(&mcu_ms5611_iodev, &sensor_ctx,
    //                                         &mcu_ms5611_iodev);
    //     if (ret < 0) {
    //       LOG_ERR("Couldn't perform read: %s", strerror(-ret));
    //     }
    //     k_sleep(K_MSEC(5000));
    // >>>>>>> main
  }
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
