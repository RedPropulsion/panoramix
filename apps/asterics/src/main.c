#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/smf.h>

#include <stddef.h>

LOG_MODULE_REGISTER(main);

struct exec_ctx {
  // MUST be the first element in the struct
  struct smf_ctx ctx;
} exec_ctx_obj;

enum state { BOOT, IDLE, CALIBRATION, MANUAL, STREAM, ARMED, LAUNCH };
// Forward declaration
const struct smf_state states[];

static enum smf_state_result boot_run(void *o) {
  LOG_ERR("Not implemented");

  smf_set_state(SMF_CTX(&exec_ctx_obj), &states[IDLE]);
  return SMF_EVENT_HANDLED;
}

static enum smf_state_result idle_run(void *o) {
  LOG_ERR("Not implemented");

  return SMF_EVENT_HANDLED;
}

const struct smf_state states[] = {
    [BOOT] = SMF_CREATE_STATE(NULL, boot_run, NULL, NULL, NULL),
    [IDLE] = SMF_CREATE_STATE(NULL, idle_run, NULL, NULL, NULL),
    [CALIBRATION] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
    [MANUAL] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
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
}
