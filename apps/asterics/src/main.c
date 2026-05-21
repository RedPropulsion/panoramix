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

static const char *const sensor_channel_name[SENSOR_CHAN_COMMON_COUNT] = {
    [SENSOR_CHAN_ACCEL_X] = "accel_x",
    [SENSOR_CHAN_ACCEL_Y] = "accel_y",
    [SENSOR_CHAN_ACCEL_Z] = "accel_z",
    [SENSOR_CHAN_ACCEL_XYZ] = "accel_xyz",
    [SENSOR_CHAN_GYRO_X] = "gyro_x",
    [SENSOR_CHAN_GYRO_Y] = "gyro_y",
    [SENSOR_CHAN_GYRO_Z] = "gyro_z",
    [SENSOR_CHAN_GYRO_XYZ] = "gyro_xyz",
    [SENSOR_CHAN_MAGN_X] = "magn_x",
    [SENSOR_CHAN_MAGN_Y] = "magn_y",
    [SENSOR_CHAN_MAGN_Z] = "magn_z",
    [SENSOR_CHAN_MAGN_XYZ] = "magn_xyz",
    [SENSOR_CHAN_DIE_TEMP] = "die_temp",
    [SENSOR_CHAN_AMBIENT_TEMP] = "ambient_temp",
    [SENSOR_CHAN_PRESS] = "press",
    [SENSOR_CHAN_PROX] = "prox",
    [SENSOR_CHAN_HUMIDITY] = "humidity",
    [SENSOR_CHAN_AMBIENT_LIGHT] = "ambient_light",
    [SENSOR_CHAN_LIGHT] = "light",
    [SENSOR_CHAN_IR] = "ir",
    [SENSOR_CHAN_RED] = "red",
    [SENSOR_CHAN_GREEN] = "green",
    [SENSOR_CHAN_BLUE] = "blue",
    [SENSOR_CHAN_ALTITUDE] = "altitude",
    [SENSOR_CHAN_PM_1_0_CF] = "pm_1_0_cf",
    [SENSOR_CHAN_PM_2_5_CF] = "pm_2_5_cf",
    [SENSOR_CHAN_PM_10_CF] = "pm_10_cf",
    [SENSOR_CHAN_PM_1_0] = "pm_1_0",
    [SENSOR_CHAN_PM_2_5] = "pm_2_5",
    [SENSOR_CHAN_PM_10] = "pm_10",
    [SENSOR_CHAN_PM_0_3_COUNT] = "pm_0_3_count",
    [SENSOR_CHAN_PM_0_5_COUNT] = "pm_0_5_count",
    [SENSOR_CHAN_PM_1_0_COUNT] = "pm_1_0_count",
    [SENSOR_CHAN_PM_2_5_COUNT] = "pm_2_5_count",
    [SENSOR_CHAN_PM_5_COUNT] = "pm_5_0_count",
    [SENSOR_CHAN_PM_10_COUNT] = "pm_10_count",
    [SENSOR_CHAN_DISTANCE] = "distance",
    [SENSOR_CHAN_CO2] = "co2",
    [SENSOR_CHAN_O2] = "o2",
    [SENSOR_CHAN_VOC] = "voc",
    [SENSOR_CHAN_GAS_RES] = "gas_resistance",
    [SENSOR_CHAN_FLOW_RATE] = "flow_rate",
    [SENSOR_CHAN_VOLTAGE] = "voltage",
    [SENSOR_CHAN_VSHUNT] = "vshunt",
    [SENSOR_CHAN_CURRENT] = "current",
    [SENSOR_CHAN_POWER] = "power",
    [SENSOR_CHAN_RESISTANCE] = "resistance",
    [SENSOR_CHAN_ROTATION] = "rotation",
    [SENSOR_CHAN_POS_DX] = "pos_dx",
    [SENSOR_CHAN_POS_DY] = "pos_dy",
    [SENSOR_CHAN_POS_DZ] = "pos_dz",
    [SENSOR_CHAN_POS_DXYZ] = "pos_dxyz",
    [SENSOR_CHAN_RPM] = "rpm",
    [SENSOR_CHAN_FREQUENCY] = "frequency",
    [SENSOR_CHAN_GAUGE_VOLTAGE] = "gauge_voltage",
    [SENSOR_CHAN_GAUGE_AVG_CURRENT] = "gauge_avg_current",
    [SENSOR_CHAN_GAUGE_STDBY_CURRENT] = "gauge_stdby_current",
    [SENSOR_CHAN_GAUGE_MAX_LOAD_CURRENT] = "gauge_max_load_current",
    [SENSOR_CHAN_GAUGE_TEMP] = "gauge_temp",
    [SENSOR_CHAN_GAUGE_STATE_OF_CHARGE] = "gauge_state_of_charge",
    [SENSOR_CHAN_GAUGE_FULL_CHARGE_CAPACITY] = "gauge_full_cap",
    [SENSOR_CHAN_GAUGE_REMAINING_CHARGE_CAPACITY] = "gauge_remaining_cap",
    [SENSOR_CHAN_GAUGE_NOM_AVAIL_CAPACITY] = "gauge_nominal_cap",
    [SENSOR_CHAN_GAUGE_FULL_AVAIL_CAPACITY] = "gauge_full_avail_cap",
    [SENSOR_CHAN_GAUGE_AVG_POWER] = "gauge_avg_power",
    [SENSOR_CHAN_GAUGE_STATE_OF_HEALTH] = "gauge_state_of_health",
    [SENSOR_CHAN_GAUGE_TIME_TO_EMPTY] = "gauge_time_to_empty",
    [SENSOR_CHAN_GAUGE_TIME_TO_FULL] = "gauge_time_to_full",
    [SENSOR_CHAN_GAUGE_CYCLE_COUNT] = "gauge_cycle_count",
    [SENSOR_CHAN_GAUGE_DESIGN_VOLTAGE] = "gauge_design_voltage",
    [SENSOR_CHAN_GAUGE_DESIRED_VOLTAGE] = "gauge_desired_voltage",
    [SENSOR_CHAN_GAUGE_DESIRED_CHARGING_CURRENT] =
        "gauge_desired_charging_current",
    [SENSOR_CHAN_GAME_ROTATION_VECTOR] = "game_rotation_vector",
    [SENSOR_CHAN_GRAVITY_VECTOR] = "gravity_vector",
    [SENSOR_CHAN_GBIAS_XYZ] = "gbias_xyz",
    [SENSOR_CHAN_ENCODER_COUNT] = "encoder_count",
    [SENSOR_CHAN_ALL] = "all",
};

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
