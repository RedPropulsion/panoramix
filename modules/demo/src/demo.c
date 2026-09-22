#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include "demo.h"

LOG_MODULE_REGISTER(demo, CONFIG_LIB_DEMO_LOG_LEVEL);

#define STRIP_NODE DT_NODELABEL(led_strip)
#define NUM_LEDS   DT_PROP(STRIP_NODE, chain_length)


// Servo demo tuning
/*
#define SERVO_ANGLE_MAX     90000   // 90 deg 
#define SERVO_SWEEP_STEP    5000    // 5 deg per sweep tick 
#define SERVO_SWEEP_MS      200
#define SERVO_WIGGLE_AMP    45000   // 45 deg wiggle amplitude 
#define SERVO_WIGGLE_MS     300

// Rest positions
#define SERVO_YAW_INIT      90000
#define SERVO_PITCH_INIT    260000 */


static const struct device *d_strip;
static struct led_rgb demo_strip[NUM_LEDS];
static struct k_work_delayable demo_neopixel_work;
static atomic_t neopixel_mode = ATOMIC_INIT(DEMO_NEOPIXEL_OFF);
static uint8_t step;

/*
static const struct device *d_pitch_servo;
static const struct device *d_yaw_servo;
static struct k_work_delayable demo_servo_work;
static int32_t yaw_angle, pitch_angle;
static int8_t yaw_dir = 1, pitch_dir = 1;
static uint8_t servo_step;
static atomic_t servo_mode = ATOMIC_INIT(DEMO_SERVO_OFF);

static void servo_go_home(void);
*/


static void demo_neopixel_work_handle(struct k_work *work)
{
    if (!d_strip) {
        return;
    }
    switch ((enum demo_neopixel_mode)atomic_get(&neopixel_mode)) {
    case DEMO_NEOPIXEL_OFF:
        memset(demo_strip, 0, sizeof(demo_strip));
        led_strip_update_rgb(d_strip, demo_strip, NUM_LEDS);
        return;
    case DEMO_NEOPIXEL_BOUNCE: {
        size_t period = 2 * (NUM_LEDS - 1);
        size_t pos = step % period;
        if (pos >= NUM_LEDS) {
            pos = period - pos;
        }
        memset(demo_strip, 0, sizeof(demo_strip));
        demo_strip[pos].r = 255;
        break;
    }
    case DEMO_NEOPIXEL_SPIN: {
        size_t prev = (step + NUM_LEDS - 1) % NUM_LEDS;
        memset(demo_strip, 0, sizeof(demo_strip));
        demo_strip[step % NUM_LEDS].r = 255;
        demo_strip[prev].r = 255;
        demo_strip[prev].g = 165;
        break;
    }
    case DEMO_NEOPIXEL_BLINK: {
        for (size_t i = 0; i < NUM_LEDS; i++) {
            uint8_t v = (step & 1) ? 128 : 0;
            demo_strip[i].r = v;
            demo_strip[i].g = v;
            demo_strip[i].b = v;
        }
        break;
    }
    default:
        return;
    }
    led_strip_update_rgb(d_strip, demo_strip, NUM_LEDS);
    step++;
    k_work_schedule(&demo_neopixel_work, K_MSEC(120));
}

void demo_neopixel_stop(void)
{
    atomic_set(&neopixel_mode, DEMO_NEOPIXEL_OFF);
    k_work_cancel_delayable(&demo_neopixel_work);
    if (d_strip) {
        memset(demo_strip, 0, sizeof(demo_strip));
        led_strip_update_rgb(d_strip, demo_strip, NUM_LEDS);
    }
}

void demo_neopixel_start(enum demo_neopixel_mode mode)
{
    demo_neopixel_stop();
    step = 0;
    atomic_set(&neopixel_mode, mode);
    k_work_schedule(&demo_neopixel_work, K_MSEC(10));
}

/* 
static void demo_servo_work_handle(struct k_work *work)
{
    switch ((enum demo_servo_mode)atomic_get(&servo_mode)) {
    case DEMO_SERVO_WIGGLE: {
        int32_t yaw_target = (servo_step & 1) ? SERVO_YAW_INIT + SERVO_WIGGLE_AMP
                                              : SERVO_YAW_INIT - SERVO_WIGGLE_AMP;
        int32_t pitch_target = (servo_step & 1) ? SERVO_PITCH_INIT + SERVO_WIGGLE_AMP
                                                : SERVO_PITCH_INIT - SERVO_WIGGLE_AMP;
        if (d_yaw_servo) {
            servo_set_position(d_yaw_servo, yaw_target);
        }
        if (d_pitch_servo) {
            servo_set_position(d_pitch_servo, pitch_target);
        }
        servo_step++;
        k_work_schedule(&demo_servo_work, K_MSEC(SERVO_WIGGLE_MS));
        break;
    }
    case DEMO_SERVO_SWEEP: {
        yaw_angle += SERVO_SWEEP_STEP * yaw_dir;
        if (yaw_angle <= SERVO_YAW_INIT) {
            yaw_angle = SERVO_YAW_INIT;
            yaw_dir = 1;
        } else if (yaw_angle >= SERVO_YAW_INIT + SERVO_ANGLE_MAX) {
            yaw_angle = SERVO_YAW_INIT + SERVO_ANGLE_MAX;
            yaw_dir = -1;
        }
        pitch_angle += SERVO_SWEEP_STEP * pitch_dir;
        if (pitch_angle <= SERVO_PITCH_INIT) {
            pitch_angle = SERVO_PITCH_INIT;
            pitch_dir = 1;
        } else if (pitch_angle >= SERVO_PITCH_INIT + SERVO_ANGLE_MAX) {
            pitch_angle = SERVO_PITCH_INIT + SERVO_ANGLE_MAX;
            pitch_dir = -1;
        }
        if (d_yaw_servo) {
            servo_set_position(d_yaw_servo, yaw_angle);
        }
        if (d_pitch_servo) {
            servo_set_position(d_pitch_servo, pitch_angle);
        }
        k_work_schedule(&demo_servo_work, K_MSEC(SERVO_SWEEP_MS));
        break;
    }
    case DEMO_SERVO_HELLO: {
        int32_t pitch_target = (servo_step & 1) ? SERVO_PITCH_INIT + SERVO_WIGGLE_AMP
                                                : SERVO_PITCH_INIT - SERVO_WIGGLE_AMP;
        if (d_pitch_servo) {
            servo_set_position(d_pitch_servo, pitch_target);
        }
        servo_step++;
        k_work_schedule(&demo_servo_work, K_MSEC(SERVO_WIGGLE_MS));
        break;
    }
    case DEMO_SERVO_OFF:
        servo_go_home();
        break;
    }
}

void demo_servo_start(enum demo_servo_mode mode)
{
    servo_step = 0;
    yaw_angle = SERVO_YAW_INIT;
    pitch_angle = SERVO_PITCH_INIT;
    yaw_dir = 1;
    pitch_dir = 1;
    atomic_set(&servo_mode, mode);
    k_work_schedule(&demo_servo_work, K_MSEC(10));
}

static void servo_go_home(void)
{
    if (d_yaw_servo) {
        servo_set_position(d_yaw_servo, SERVO_YAW_INIT);
    }
    if (d_pitch_servo) {
        servo_set_position(d_pitch_servo, SERVO_PITCH_INIT);
    }
}

void demo_servo_stop(void)
{
    atomic_set(&servo_mode, DEMO_SERVO_OFF);
    k_work_cancel_delayable(&demo_servo_work);
    servo_go_home();
}
    
*/

void demo_off(void)
{
    //demo_servo_stop();
    demo_neopixel_stop();
}

bool demo_active(void)
{
    //return (atomic_get(&neopixel_mode) != DEMO_NEOPIXEL_OFF) || (atomic_get(&servo_mode) != DEMO_SERVO_OFF);
    return atomic_get(&neopixel_mode) != DEMO_NEOPIXEL_OFF;
}

int demo_init(void)
{
    d_strip = DEVICE_DT_GET(STRIP_NODE);
    if (device_is_ready(d_strip)) {
        memset(demo_strip, 0, sizeof(demo_strip));
        led_strip_update_rgb(d_strip, demo_strip, NUM_LEDS);
        LOG_INF("Demo: LED strip ready");
    } else {
        LOG_WRN("Demo: LED strip not ready");
        d_strip = NULL;
    }
    /*
    
    d_pitch_servo = DEVICE_DT_GET(DT_NODELABEL(pitch_servo));
    d_yaw_servo = DEVICE_DT_GET(DT_NODELABEL(yaw_servo));
    if (device_is_ready(d_pitch_servo)) {
        LOG_INF("Demo: Pitch servo ready");
        servo_set_position(d_pitch_servo, SERVO_PITCH_INIT);
    } else {
        LOG_WRN("Demo: Pitch servo not ready");
        d_pitch_servo = NULL;
    }
    if (device_is_ready(d_yaw_servo)) {
        LOG_INF("Demo: Yaw servo ready");
        servo_set_position(d_yaw_servo, SERVO_YAW_INIT);
    } else {
        LOG_WRN("Demo: Yaw servo not ready");
        d_yaw_servo = NULL;
    }
    
    */
    k_work_init_delayable(&demo_neopixel_work, demo_neopixel_work_handle);
    //k_work_init_delayable(&demo_servo_work, demo_servo_work_handle);
#if defined(CONFIG_LIB_MENU)
    demo_menu.parent = menu_get_main();
#endif
    return 0;
}

#if defined(CONFIG_LIB_MENU)
static void menu_neopixel_bounce(void) { demo_neopixel_start(DEMO_NEOPIXEL_BOUNCE); }
static void menu_neopixel_spin(void)   { demo_neopixel_start(DEMO_NEOPIXEL_SPIN); }
static void menu_neopixel_blink(void)  { demo_neopixel_start(DEMO_NEOPIXEL_BLINK); }
//static void menu_servo_wiggle(void)    { demo_servo_start(DEMO_SERVO_WIGGLE); }
//static void menu_servo_sweep(void)     { demo_servo_start(DEMO_SERVO_SWEEP); }
//static void menu_servo_hello(void)     { demo_servo_start(DEMO_SERVO_HELLO); }

static struct menu_item neopixel_items[] = {
    {"Bounce",       NULL, menu_neopixel_bounce, NULL, false},
    {"Spin",         NULL, menu_neopixel_spin,   NULL, false},
    {"Blink",        NULL, menu_neopixel_blink,  NULL, false},
    {"Neopixel OFF", NULL, demo_neopixel_stop,   NULL, false},
};

/*
static struct menu_item servo_items[] = {
    {"Wiggle",    NULL, menu_servo_wiggle, NULL, false},
    {"Sweep",     NULL, menu_servo_sweep,  NULL, false},
    {"Hello!",    NULL, menu_servo_hello,  NULL, false},
    {"Servo OFF", NULL, demo_servo_stop,   NULL, false},
};
*/

static struct menu demo_neopixels_menu = {
    .title = "Neopixels LEDs",
    .items = neopixel_items,
    .item_count = ARRAY_SIZE(neopixel_items),
    .parent = &demo_menu,
};

/*
static struct menu demo_servo_menu = {
    .title = "Servos ST3215",
    .items = servo_items,
    .item_count = ARRAY_SIZE(servo_items),
    .parent = &demo_menu,
};
*/

static struct menu_item demo_items[] = {
    {"Light LEDs",  NULL, NULL, &demo_neopixels_menu, false},
    //{"Move Servos", NULL, NULL, &demo_servo_menu,     false},
    {"Demo OFF",    NULL, demo_off, NULL, false},
};
struct menu demo_menu = {
    .title = "Demo",
    .items = demo_items,
    .item_count = ARRAY_SIZE(demo_items),
    .parent = NULL, /* set to menu_get_main() in demo_init() */
};
#endif /* CONFIG_LIB_MENU */