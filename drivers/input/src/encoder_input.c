#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(encoder_input, LOG_LEVEL_DBG);

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#include "encoder_input.h"

#define DEBOUNCE_ROTATE_MS  5
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

static uint32_t last_a_time = 0;
static uint32_t last_s_time = 0;
static bool waiting_double_press = false;
static uint32_t first_press_time = 0;
static bool last_switch_state = true;

static void encoder_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    uint32_t now = k_uptime_get_32();

    if (now - last_a_time < DEBOUNCE_ROTATE_MS) {
        return;
    }
    last_a_time = now;

    int phase_sw = gpio_pin_get(enc_s_port, enc_s_pin);
    int phase_b = gpio_pin_get(enc_b_port, enc_b_pin);

    enum encoder_event ev;

    if (!phase_sw) {
        ev = phase_b ? ENCODER_PRESS_ROTATE_CCW : ENCODER_PRESS_ROTATE_CW;
    } else {
        ev = phase_b ? ENCODER_ROTATE_CCW : ENCODER_ROTATE_CW;
    }

    LOG_DBG("Encoder rotation: phase_b=%d switch=%d -> %d", phase_b, phase_sw, ev);

    int ret = k_msgq_put(&encoder_msgq, &ev, K_NO_WAIT);
    if (ret != 0) {
        LOG_DBG("Encoder msgq full, dropped event");
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
        if (!waiting_double_press) {
            first_press_time = now;
            waiting_double_press = true;
        } else if (now - first_press_time < DOUBLE_PRESS_MS) {
            enum encoder_event ev = ENCODER_DOUBLE_PRESS;
            k_msgq_put(&encoder_msgq, &ev, K_NO_WAIT);
            waiting_double_press = false;
        }
    } else {
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

    ret = gpio_pin_interrupt_configure(enc_a_port, enc_a_pin, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure encoder A interrupt: %s %d", strerror(ret), ret);
        return ret;
    }

    gpio_init_callback(&enc_a_cb_data, encoder_handler, BIT(enc_a_pin));
    ret = gpio_add_callback(enc_a_port, &enc_a_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add encoder A callback: %s %d", strerror(ret), ret);
        return ret;
    }

    k_timer_start(&switch_poll_timer, K_MSEC(10), K_NO_WAIT);

    LOG_INF("Encoder input initialized (direct GPIO interrupts)");
    return 0;
}
