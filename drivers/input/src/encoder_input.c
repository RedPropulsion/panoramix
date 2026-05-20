#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(encoder_input, CONFIG_ENCODER_INPUT_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#include "encoder_input.h"

#define DEBOUNCE_SWITCH_MS  50
#define DOUBLE_PRESS_MS     300

K_MSGQ_DEFINE(encoder_msgq, sizeof(enum encoder_event), ENCODER_MSGQ_SIZE, 4);

static const struct device *const enc_a_port = DEVICE_DT_GET(DT_NODELABEL(gpiod));
static const struct device *const enc_b_port = DEVICE_DT_GET(DT_NODELABEL(gpiod));
static const struct device *const enc_s_port = DEVICE_DT_GET(DT_NODELABEL(gpiog));
static const uint8_t enc_a_pin = 0;
static const uint8_t enc_b_pin = 1;
static const uint8_t enc_s_pin = 0;

static struct gpio_callback enc_a_cb_data;
static struct gpio_callback enc_b_cb_data;

static uint8_t last_ab_state;
static int8_t enc_delta;
static uint32_t last_s_time = 0;
static bool waiting_double_press = false;
static uint32_t first_press_time = 0;
static bool last_switch_state = true;

/*
 * Quadrature state transition table.
 * Index = (prev_state << 2) | new_state
 * Value: +1 = CW step, -1 = CCW step, 0 = invalid/no change
 *
 * CW sequence:  00 → 10 → 11 → 01 → 00
 * CCW sequence: 00 → 01 → 11 → 10 → 00
 */
static const int8_t enc_table[16] = {
     0, -1, +1,  0,   /* 00 → 00, 01, 10, 11 */
    +1,  0,  0, -1,   /* 01 → 00, 01, 10, 11 */
    -1,  0,  0, +1,   /* 10 → 00, 01, 10, 11 */
     0, +1, -1,  0,   /* 11 → 00, 01, 10, 11 */
};

static void emit_encoder_event(enum encoder_event ev)
{
    const char *ev_name[] = {
        [ENCODER_ROTATE_CW] = "ROTATE_CW",
        [ENCODER_ROTATE_CCW] = "ROTATE_CCW",
        [ENCODER_DOUBLE_PRESS] = "DOUBLE_PRESS",
        [ENCODER_PRESS_ROTATE_CW] = "PRESS_ROTATE_CW",
        [ENCODER_PRESS_ROTATE_CCW] = "PRESS_ROTATE_CCW",
    };
    LOG_INF("Encoder EVT: %s", ev_name[ev]);

    int ret = k_msgq_put(&encoder_msgq, &ev, K_NO_WAIT);
    if (ret != 0) {
        LOG_WRN("Encoder msgq full, dropped %s", ev_name[ev]);
    }
}

static void encoder_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    int phase_a = gpio_pin_get(enc_a_port, enc_a_pin);
    int phase_b = gpio_pin_get(enc_b_port, enc_b_pin);
    int phase_sw = gpio_pin_get(enc_s_port, enc_s_pin);

    uint8_t new_state = (phase_a << 1) | phase_b;
    int8_t delta = enc_table[(last_ab_state << 2) | new_state];

    if (delta != 0) {
        last_ab_state = new_state;
        enc_delta += delta;

        if (enc_delta >= 4) {
            enc_delta -= 4;
            emit_encoder_event(phase_sw ? ENCODER_ROTATE_CW : ENCODER_PRESS_ROTATE_CW);
        } else if (enc_delta <= -4) {
            enc_delta += 4;
            emit_encoder_event(phase_sw ? ENCODER_ROTATE_CCW : ENCODER_PRESS_ROTATE_CCW);
        }
    }
}

static void switch_poll_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint32_t now = k_uptime_get_32();
    bool pressed = gpio_pin_get(enc_s_port, enc_s_pin) == 0;

    if (pressed == last_switch_state) {
        return;
    }

    if (now - last_s_time < DEBOUNCE_SWITCH_MS) {
        return;
    }
    last_s_time = now;
    last_switch_state = pressed;

    if (pressed) {
        LOG_INF("Encoder EVT: SWITCH_PRESS (waiting_double=%d)", waiting_double_press);
        if (!waiting_double_press) {
            first_press_time = now;
            waiting_double_press = true;
        } else if (now - first_press_time < DOUBLE_PRESS_MS) {
            enum encoder_event ev = ENCODER_DOUBLE_PRESS;
            LOG_INF("Encoder EVT: DOUBLE_PRESS");
            k_msgq_put(&encoder_msgq, &ev, K_NO_WAIT);
            waiting_double_press = false;
        }
    } else {
        LOG_INF("Encoder EVT: SWITCH_RELEASE (waiting_double=%d elapsed=%dms)",
                waiting_double_press, now - first_press_time);
        if (waiting_double_press && (now - first_press_time >= DOUBLE_PRESS_MS)) {
            waiting_double_press = false;
        }
    }
}

static K_WORK_DELAYABLE_DEFINE(switch_poll_work, switch_poll_work_handler);

static void switch_poll_timer_handler(struct k_timer *timer)
{
    k_work_reschedule(&switch_poll_work, K_NO_WAIT);
    k_timer_start(timer, K_MSEC(10), K_NO_WAIT);
}

static K_TIMER_DEFINE(switch_poll_timer, switch_poll_timer_handler, NULL);

int encoder_input_init(void)
{
    int ret;

    if (!device_is_ready(enc_a_port)) {
        LOG_ERR("GPIOD not ready");
        return -ENODEV;
    }
    if (!device_is_ready(enc_b_port)) {
        LOG_ERR("GPIOD not ready");
        return -ENODEV;
    }
    if (!device_is_ready(enc_s_port)) {
        LOG_ERR("GPIOG not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure(enc_a_port, enc_a_pin, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure encoder A: %s %d", strerror(ret), ret);
        return ret;
    }

    ret = gpio_pin_configure(enc_b_port, enc_b_pin, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure encoder B: %s %d", strerror(ret), ret);
        return ret;
    }

    ret = gpio_pin_configure(enc_s_port, enc_s_pin, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure encoder switch: %s %d", strerror(ret), ret);
        return ret;
    }

    last_ab_state = (gpio_pin_get(enc_a_port, enc_a_pin) << 1) | gpio_pin_get(enc_b_port, enc_b_pin);
    enc_delta = 0;

    ret = gpio_pin_interrupt_configure(enc_a_port, enc_a_pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure encoder A interrupt: %s %d", strerror(ret), ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure(enc_b_port, enc_b_pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure encoder B interrupt: %s %d", strerror(ret), ret);
        return ret;
    }

    gpio_init_callback(&enc_a_cb_data, encoder_handler, BIT(enc_a_pin));
    ret = gpio_add_callback(enc_a_port, &enc_a_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add encoder A callback: %s %d", strerror(ret), ret);
        return ret;
    }

    gpio_init_callback(&enc_b_cb_data, encoder_handler, BIT(enc_b_pin));
    ret = gpio_add_callback(enc_b_port, &enc_b_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add encoder B callback: %s %d", strerror(ret), ret);
        return ret;
    }

    k_timer_start(&switch_poll_timer, K_MSEC(10), K_NO_WAIT);

    LOG_INF("Encoder input initialized (quadrature state machine)");
    return 0;
}
