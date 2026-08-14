#include <stdint.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(menu, LOG_LEVEL_INF);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/servo.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "menu.h"
#include <encoder_input.h>

#define MIN_REDRAW_MS CONFIG_LIB_MENU_MIN_REDRAW_MS
#define STRIP_NODE DT_NODELABEL(led_strip)
#define NUM_LEDS DT_PROP(STRIP_NODE, chain_length)


K_SEM_DEFINE(menu_data_sem, 0, 1);

struct menu_display_data menu_data = {0};

static struct menu_state state = {0};
static const struct device *disp;
static uint32_t last_redraw = 0;

static struct menu demo_neopixels_menu;
static const struct device* d_strip;
static struct led_rgb demo_strip[NUM_LEDS];
static struct k_work_delayable demo_neopixel_work;

static struct menu demo_servo_menu;
static const struct device* d_pitch_servo;
static const struct device* d_yaw_servo;
static struct k_work_delayable demo_servo_work;
static int32_t yaw_angle, pitch_angle;
static int8_t yaw_dir = 1, pitch_dir = 1;
static uint8_t servo_step;

enum demo_servo_mode {
    SERVO_NONE,
    SERVO_WIGGLE,
    SERVO_SWEEP,
    SERVO_HELLO,
};

static enum demo_servo_mode servo_mode = SERVO_NONE;

// Servo demo tuning
#define SERVO_ANGLE_MAX 90000    // 90 deg
#define SERVO_SWEEP_STEP 5000    // 5 deg per sweep tick 
#define SERVO_SWEEP_MS 200
#define SERVO_WIGGLE_AMP 45000   // 45 deg wiggle amplitude 
#define SERVO_WIGGLE_MS 300

// rest position per axis (mdeg)
#define SERVO_YAW_INIT 90000
#define SERVO_PITCH_INIT 260000

enum demo_neopixel_mode{
    NONE,
    SPIN, 
    BLINK,
    BOUNCE,
};

static enum demo_neopixel_mode neopixel_mode = NONE;
static uint8_t step;

static struct k_poll_event events[2];

static void draw_row(uint8_t row, const char *str, bool invert);
static void clear_display(void);
static void menu_redraw(void);
static void menu_handle_event(enum encoder_event evt);
static void update_scroll(void);
static void servo_go_home(void);

static struct menu main_menu;
static struct menu commands_menu;
static struct menu demo_menu;

static void draw_row(uint8_t row, const char *str, bool invert)
{
    char buf[17] = {0};
    snprintf(buf, sizeof(buf), "%-16s", str);

    // if (invert) {
    //     cfb_invert_area(disp, 0, row * 8, 128, 8);
    // }

    cfb_print(disp, buf, 0, row * 8);

    if (invert) {
        cfb_invert_area(disp, 0, row * 8, 128, 8);
    }
}

static void clear_display(void)
{
    cfb_framebuffer_clear(disp, true);
}

static void update_scroll(void)
{
    if (state.current->item_count <= MENU_VISIBLE_ROWS) {
        state.scroll_offset = 0;
        return;
    }

    size_t cursor_virtual_pos = state.selected;

    if (cursor_virtual_pos < MENU_CURSOR_ROW) {
        state.scroll_offset = 0;
    } else if (cursor_virtual_pos > state.current->item_count - (MENU_VISIBLE_ROWS - MENU_CURSOR_ROW)) {
        state.scroll_offset = state.current->item_count - MENU_VISIBLE_ROWS;
    } else {
        state.scroll_offset = cursor_virtual_pos - MENU_CURSOR_ROW;
    }
}

static void draw_confirmation_screen(void)
{
    clear_display();

    if (state.confirming_item) {
        draw_row(0, "Confirm?", false);
        draw_row(1, state.confirming_item->label, false);
        draw_row(2, "", false);
        draw_row(3, "Yes: Double-press", false);
        draw_row(4, "No: Press+Rotate<", false);
    }

    cfb_framebuffer_finalize(disp);
}

static void draw_menu_screen(void)
{
    // clear_display();

    if (state.current->title) {
        draw_row(0, state.current->title, false);
    }

    size_t start = state.scroll_offset;
    size_t end = start + MENU_VISIBLE_ROWS;
    if (end > state.current->item_count) {
        end = state.current->item_count;
    }

    for (size_t i = start; i < end; i++) {
        size_t row = i - start + 1;
        bool is_selected = (i == state.selected);

        char buf[17] = {0};
        struct menu_item *item = &state.current->items[i];

        snprintf(buf, sizeof(buf), " %s", item->label);

        draw_row(row, buf, is_selected);
    }

    cfb_framebuffer_finalize(disp);
}

static void menu_redraw(void)
{
    if (state.showing_confirmation) {
        draw_confirmation_screen();
    } else if (state.active_draw_fn) {
        state.active_draw_fn();
    } else {
        draw_menu_screen();
    }
    last_redraw = k_uptime_get_32();
}

static void menu_handle_event(enum encoder_event evt)
{
    if (state.showing_confirmation) {
        switch (evt) {
        case ENCODER_DOUBLE_PRESS:
            if (state.confirming_item && state.confirming_item->action_fn) {
                state.confirming_item->action_fn();
            }
            state.showing_confirmation = false;
            state.confirming_item = NULL;
            break;
        case ENCODER_PRESS_ROTATE_CCW:
            state.showing_confirmation = false;
            state.confirming_item = NULL;
            break;
        default:
            break;
        }
        return;
    }

    switch (evt) {
    case ENCODER_ROTATE_CW:
        if (state.selected < state.current->item_count - 1) {
            state.selected++;
            update_scroll();
        }
        break;
    case ENCODER_ROTATE_CCW:
        if (state.selected > 0) {
            state.selected--;
            update_scroll();
        }
        break;
    case ENCODER_DOUBLE_PRESS: {
        struct menu_item *item = &state.current->items[state.selected];
        if (item->draw_fn) {
            state.active_draw_fn = item->draw_fn;
            state.active_title = item->label;
            clear_display();
        } else if (item->submenu) {
            state.current = item->submenu;
            state.selected = 0;
            state.scroll_offset = 0;
        } else if (item->action_fn) {
            if (item->needs_confirmation) {
                state.showing_confirmation = true;
                state.confirming_item = item;
            } else {
                item->action_fn();
            }
}
        break;
    }
    case ENCODER_PRESS_ROTATE_CW: {
        if (state.current->item_count == 0 || state.selected >= state.current->item_count) {
            break;
        }
        struct menu_item *item = &state.current->items[state.selected];
        if (item->draw_fn) {
            state.active_draw_fn = item->draw_fn;
            state.active_title = item->label;
            clear_display();
        } else if (item->submenu) {
            state.current = item->submenu;
            state.selected = 0;
            state.scroll_offset = 0;
            clear_display();
        }
        break;
    }
    case ENCODER_PRESS_ROTATE_CCW:
        if (state.active_draw_fn) {
            state.active_draw_fn = NULL;
            state.active_title = NULL;
            clear_display();
        } else if (state.current->parent) {
            state.current = state.current->parent;
            state.selected = 0;
            state.scroll_offset = 0;
            clear_display();
        }
        break;
    }
}

void menu_thread(void *p1, void *p2, void *p3)
{
    k_poll_event_init(&events[0], K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                      K_POLL_MODE_NOTIFY_ONLY, &encoder_msgq);
    k_poll_event_init(&events[1], K_POLL_TYPE_SEM_AVAILABLE,
                      K_POLL_MODE_NOTIFY_ONLY, &menu_data_sem);

while (1) {
        k_poll(events, 2, K_FOREVER);

        bool user_input = false;
        bool data_changed = false;

        if (events[0].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
            enum encoder_event evt;
            k_msgq_get(&encoder_msgq, &evt, K_NO_WAIT);
            menu_handle_event(evt);
            user_input = true;
            events[0].state = K_POLL_STATE_NOT_READY;
        }

        if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
            k_sem_take(&menu_data_sem, K_NO_WAIT);
            data_changed = true;
            events[1].state = K_POLL_STATE_NOT_READY;
        }

        uint32_t now = k_uptime_get_32();
        bool should_redraw = user_input ||
                            (data_changed && (now - last_redraw >= MIN_REDRAW_MS));

        if (should_redraw) {
            menu_redraw();
            last_redraw = now;
        }
    }
}

K_THREAD_STACK_DEFINE(menu_stack, CONFIG_LIB_MENU_STACK_SIZE);
static struct k_thread menu_thread_data;

void menu_start(void)
{
    k_thread_create(&menu_thread_data, menu_stack,
                    K_THREAD_STACK_SIZEOF(menu_stack),
                    menu_thread, NULL, NULL, NULL,
                    CONFIG_LIB_MENU_PRIORITY, 0, K_NO_WAIT);
}

void menu_update_lora_stats(uint32_t tx, uint32_t rx, int16_t rssi, int8_t snr)
{
    atomic_set(&menu_data.lora.tx_count, tx);
    atomic_set(&menu_data.lora.rx_count, rx);
    atomic_set(&menu_data.lora.last_rssi, rssi);
    atomic_set(&menu_data.lora.last_snr, snr);
    k_sem_give(&menu_data_sem);
}

void menu_update_udp_stats(uint32_t tx, uint32_t rx)
{
    atomic_set(&menu_data.udp.tx_count, tx);
    atomic_set(&menu_data.udp.rx_count, rx);
    k_sem_give(&menu_data_sem);
}

void menu_update_gps(bool valid, uint8_t sats, uint8_t fix, int32_t lat, int32_t lon, int32_t alt)
{
    menu_data.gps.valid = valid;
    menu_data.gps.satellites = sats;
    menu_data.gps.fix_type = fix;
    menu_data.gps.latitude = lat;
    menu_data.gps.longitude = lon;
    menu_data.gps.altitude_mm = alt;
    k_sem_give(&menu_data_sem);
}

void menu_update_asterics(const char *mode, uint16_t battery_mv, bool armed, uint8_t state)
{
    menu_data.asterics.flight_mode = mode;
    menu_data.asterics.battery_mv = battery_mv;
    menu_data.asterics.armed = armed;
    menu_data.asterics.state = state;
    k_sem_give(&menu_data_sem);
}

static void draw_comms_screen(void)
{
    clear_display();
    draw_row(0, "COMMS", false);
    draw_row(1, "", false);

    char buf[17];
    snprintf(buf, sizeof(buf), "LoRa TX:%u RX:%u",
             (int32_t)atomic_get(&menu_data.lora.tx_count),
             (int32_t)atomic_get(&menu_data.lora.rx_count));
    draw_row(2, buf, false);

    snprintf(buf, sizeof(buf), "RSSI:%d SNR:%d",
             (int32_t)atomic_get(&menu_data.lora.last_rssi),
             (int32_t)atomic_get(&menu_data.lora.last_snr));
    draw_row(3, buf, false);

    snprintf(buf, sizeof(buf), "UDP TX:%u RX:%u",
             (int32_t)atomic_get(&menu_data.udp.tx_count),
             (int32_t)atomic_get(&menu_data.udp.rx_count));
    draw_row(4, buf, false);

    cfb_framebuffer_finalize(disp);
}

static void draw_gps_screen(void)
{
    clear_display();
    draw_row(0, "GPS (MAVLink)", false);
    draw_row(1, "", false);

    char buf[17];
    snprintf(buf, sizeof(buf), "Sats:%u Fix:%u",
             menu_data.gps.satellites, menu_data.gps.fix_type);
    draw_row(2, buf, false);

    if (menu_data.gps.valid) {
        int32_t lat = menu_data.gps.latitude;
        int32_t lon = menu_data.gps.longitude;
        snprintf(buf, sizeof(buf), "Lat:%d.%06u", lat / 10000000, abs(lat % 10000000));
        draw_row(3, buf, false);
        snprintf(buf, sizeof(buf), "Lon:%d.%06u", lon / 10000000, abs(lon % 10000000));
        draw_row(4, buf, false);
        snprintf(buf, sizeof(buf), "Alt:%dm", menu_data.gps.altitude_mm / 1000);
        draw_row(5, buf, false);
    } else {
        draw_row(3, "No GPS data", false);
    }

    cfb_framebuffer_finalize(disp);
}

static void draw_status_screen(void)
{
    clear_display();
    draw_row(0, "Asterics Status", false);
    draw_row(1, "", false);

    char buf[17];
    snprintf(buf, sizeof(buf), "Mode:%s",
             menu_data.asterics.flight_mode ? menu_data.asterics.flight_mode : "N/A");
    draw_row(2, buf, false);

    snprintf(buf, sizeof(buf), "Armed:%s",
             menu_data.asterics.armed ? "Yes" : "No");
    draw_row(3, buf, false);

    snprintf(buf, sizeof(buf), "Batt:%u.%02uV",
             menu_data.asterics.battery_mv / 1000,
             (menu_data.asterics.battery_mv % 1000) / 10);
    draw_row(4, buf, false);

    cfb_framebuffer_finalize(disp);
}

static void cmd_arm_disarm(void)
{
    LOG_INF("MAVLink: Would send ARM/DISARM command");
}

static void cmd_request_telemetry(void)
{
    LOG_INF("MAVLink: Would send Request Telemetry");
}

static void cmd_set_lora_channel(void)
{
    LOG_INF("MAVLink: Would send Set LoRa Channel");
}    

static void cmd_reboot(void)
{
    LOG_INF("MAVLink: Would send Reboot");
}    

/* 
 *  ------------------------------------------------------------------
 *  
 *      DEMO SECTION BEGIN
 * 
 *  ------------------------------------------------------------------ 
*/ 


static void demo_neopixel_work_handle(struct k_work* work){
    if(!d_strip){
        return;
    }    

    switch(neopixel_mode){
        case NONE:{
            memset(demo_strip, 0, sizeof(demo_strip));
            led_strip_update_rgb(d_strip, demo_strip, NUM_LEDS);
            return;
        }    
        case BOUNCE:{
            size_t period = 2 * (NUM_LEDS - 1 );
            size_t pos = step % period;
            if(pos >= NUM_LEDS)
                pos = period - pos;
            memset(demo_strip, 0, sizeof(demo_strip));    
            demo_strip[pos].r = 255;
            break;
        }    
        case SPIN:{
            size_t prev = (step + NUM_LEDS - 1) % NUM_LEDS;
            memset(demo_strip, 0, sizeof(demo_strip));
            demo_strip[step % NUM_LEDS].r = 255;
            demo_strip[prev].r = 255; //orange follows red
            demo_strip[prev].g = 165;
            break;
        }    
        case BLINK:{
            for(size_t i = 0; i < NUM_LEDS; i++){
                uint8_t v = (step & 1) ? 128 : 0;
                demo_strip[i].r = v;
                demo_strip[i].g = v;
                demo_strip[i].b = v;
            }    
            break;
        }    
        default: return;
    }    

    led_strip_update_rgb(d_strip, demo_strip, NUM_LEDS);
    step++;
    k_work_schedule(&demo_neopixel_work, K_MSEC(120));
}    


static void demo_neopixel_stop(void)
{
    neopixel_mode = NONE;
    k_work_cancel_delayable(&demo_neopixel_work);
    if (d_strip) {
        memset(demo_strip, 0, sizeof(demo_strip));
        led_strip_update_rgb(d_strip, demo_strip, NUM_LEDS);
    }
}

static void neopixel_bounce(void){
    demo_neopixel_stop();
    neopixel_mode = BOUNCE;
    step = 0;
    k_work_schedule(&demo_neopixel_work, K_MSEC(10));
}

static void neopixel_spin(void){
    demo_neopixel_stop();
    neopixel_mode = SPIN;
    step = 0;
    k_work_schedule(&demo_neopixel_work, K_MSEC(10));
}

static void neopixel_blink(void){
    demo_neopixel_stop();
    neopixel_mode = BLINK;
    step = 0;
    k_work_schedule(&demo_neopixel_work, K_MSEC(10));
}

static void neopixel_off(void){
    demo_neopixel_stop();
    neopixel_mode = NONE;
    step = 0;
    k_work_schedule(&demo_neopixel_work, K_MSEC(10));
}

static void demo_servo_work_handle(struct k_work *work)
{
    switch (servo_mode) {
    case SERVO_WIGGLE: {
        int32_t yaw_target =
        (servo_step & 1) ? SERVO_YAW_INIT + SERVO_WIGGLE_AMP
        : SERVO_YAW_INIT - SERVO_WIGGLE_AMP;
        int32_t pitch_target =
            (servo_step & 1) ? SERVO_PITCH_INIT + SERVO_WIGGLE_AMP
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
    case SERVO_SWEEP: {
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
    case SERVO_HELLO: {
        int32_t pitch_target = (servo_step & 1) ? SERVO_PITCH_INIT + SERVO_WIGGLE_AMP
                                                : SERVO_PITCH_INIT - SERVO_WIGGLE_AMP;
        if (d_pitch_servo)
            servo_set_position(d_pitch_servo, pitch_target);
        servo_step++;
        k_work_schedule(&demo_servo_work, K_MSEC(SERVO_WIGGLE_MS));
        break;
    }
    case SERVO_NONE:
        servo_go_home();
        break;
    }
}

static void demo_servo_start()
{
    servo_step = 0;
    yaw_angle = SERVO_YAW_INIT;
    pitch_angle = SERVO_PITCH_INIT;
    yaw_dir = 1;
    pitch_dir = 1;
    k_work_schedule(&demo_servo_work, K_MSEC(10));
}

static void servo_wiggle(void)
{
    servo_mode = SERVO_WIGGLE;
    demo_servo_start();
}

static void servo_sweep(void)
{
    servo_mode = SERVO_SWEEP;
    demo_servo_start();
}

static void servo_hello(void){
    servo_mode = SERVO_HELLO;
    demo_servo_start();
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

static void demo_servo_stop(void)
{
    servo_mode = SERVO_NONE;
    k_work_cancel_delayable(&demo_servo_work);
    servo_go_home();
}

static void demo_init(void){
    d_strip = DEVICE_DT_GET(STRIP_NODE);
    if(device_is_ready(d_strip)){
        memset(demo_strip, 0, sizeof(demo_strip));
        led_strip_update_rgb(d_strip, demo_strip, NUM_LEDS);
        LOG_INF("Demo: LED strip ready");
    }else{
        LOG_WRN("Demo: LED strip not ready");
        d_strip = NULL;
    }
    d_pitch_servo = DEVICE_DT_GET(DT_NODELABEL(pitch_servo));
    d_yaw_servo = DEVICE_DT_GET(DT_NODELABEL(yaw_servo));
    if(device_is_ready(d_pitch_servo)){
        LOG_INF("Demo: Pitch servo ready");
        servo_set_position(d_pitch_servo, SERVO_PITCH_INIT);
    }else{
        LOG_WRN("Demo: Pitch servo not read");
        d_pitch_servo = NULL;
    }
    if(device_is_ready(d_yaw_servo)){
        LOG_INF("Demo: Yaw servo ready");
        servo_set_position(d_yaw_servo, SERVO_YAW_INIT);
    }else{
        LOG_WRN("Demo: Yaw servo not read");
        d_yaw_servo = NULL;
    }
    k_work_init_delayable(&demo_neopixel_work, demo_neopixel_work_handle);
    k_work_init_delayable(&demo_servo_work, demo_servo_work_handle);
}

bool menu_demo_active(void){
    return neopixel_mode != NONE;
}

static void demo_off(void){
    demo_servo_stop();
    demo_neopixel_stop();
}

static struct menu_item neopixel_items[] = {
    {"Bounce", NULL, neopixel_bounce, NULL, false },
    {"Spin", NULL, neopixel_spin, NULL, false},
    {"Blink", NULL, neopixel_blink, NULL, false},
    {"Neopixel OFF", NULL, neopixel_off, NULL, false},
};

static struct menu_item servo_items[] = {
    {"Wiggle", NULL, servo_wiggle, NULL, false},
    {"Sweep", NULL, servo_sweep, NULL, false},
    {"Hello!", NULL, servo_hello, NULL, false},
    {"Servo OFF", NULL, demo_servo_stop, NULL, false},
};

static struct menu_item demo_items[] = {
    {"Light LEDs", NULL, NULL, &demo_neopixels_menu, false},
    {"Move Servos", NULL, NULL, &demo_servo_menu, false},
    {"Demo OFF", NULL, demo_off, NULL, false},
};

static struct menu demo_menu = {
    .title = "Demo",
    .items = demo_items, 
    .item_count = ARRAY_SIZE(demo_items),
    .parent = &main_menu,
};

static struct menu demo_neopixels_menu = {
    .title = "Neopixels LEDs",
    .items = neopixel_items,
    .item_count = ARRAY_SIZE(neopixel_items),
    .parent = &demo_menu,
};

static struct menu demo_servo_menu = {
    .title = "Servos ST3215",
    .items = servo_items,
    .item_count = ARRAY_SIZE(servo_items),
    .parent = &demo_menu,
};


/* ---------------------------------------------------------------------
 *
 *      DEMO SECTION END
 *
 * ---------------------------------------------------------------------
*/

static struct menu_item commands_items[] = {
    {"Arm/Disarm", NULL, cmd_arm_disarm, NULL, true},
    {"Request Telemetry", NULL, cmd_request_telemetry, NULL, false},
    {"Set LoRa Channel", NULL, cmd_set_lora_channel, NULL, true},
    {"Reboot", NULL, cmd_reboot, NULL, true},
};


static struct menu_item main_items[] = {
    {"COMMS",    draw_comms_screen,  NULL, NULL, false},
    {"GPS",      draw_gps_screen,    NULL, NULL, false},
    {"Status",   draw_status_screen, NULL, NULL, false},
    {"Commands", NULL,               NULL, &commands_menu, false},
    {"Demo", NULL, NULL, &demo_menu, false},
};

static struct menu main_menu = {
    .title = "Main",
    .items = main_items,
    .item_count = ARRAY_SIZE(main_items),
    .parent = NULL,
};

static struct menu commands_menu = {
    .title = "Commands",
    .items = commands_items,
    .item_count = ARRAY_SIZE(commands_items),
    .parent = &main_menu,
};


static void display_init(void)
{
    disp = DEVICE_DT_GET(DT_NODELABEL(ssd1309));
    if (!device_is_ready(disp)) {
        LOG_ERR("Display not ready");
        return;
    }



    if (display_set_pixel_format(disp, PIXEL_FORMAT_MONO10) != 0) {
        display_set_pixel_format(disp, PIXEL_FORMAT_MONO01);
    }

    cfb_framebuffer_init(disp);
    cfb_framebuffer_clear(disp, true);
    display_blanking_off(disp);

    LOG_INF("Display initialized for menu");
}

int menu_init(void)
{
    encoder_input_init();

    display_init();
    demo_init();

    state.current = &main_menu;
    state.selected = 0;
    state.scroll_offset = 0;
    state.showing_confirmation = false;
    state.confirming_item = NULL;
    state.active_draw_fn = NULL;
    state.active_title = NULL;

    menu_update_asterics("STANDBY", 12400, false, 0);
    menu_update_gps(false, 0, 0, 0, 0, 0);

    LOG_INF("Menu initialized");
    return 0;
}