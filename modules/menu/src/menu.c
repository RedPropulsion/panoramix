#include <common/mavlink.h>
#include <common/mavlink_msg_command_long.h>
#include <common/mavlink_msg_ping.h>
#include <mavlink_types.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(menu, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <mavwrap.h>

#include "menu.h"
#include <encoder_input.h>


#define MIN_REDRAW_MS CONFIG_LIB_MENU_MIN_REDRAW_MS

K_SEM_DEFINE(menu_data_sem, 0, 1);

struct cmd_devices {
    struct device *mav_lora;
    struct device *mav_udp;
}__mav_devices;

struct menu_display_data menu_data = {0};

static struct menu_state state = {0};
static const struct device *disp;
static uint32_t last_redraw = 0;

static struct k_poll_event events[2];

static void draw_row(uint8_t row, const char *str, bool invert);
static void clear_display(void);
static void menu_redraw(void);
static void menu_handle_event(enum encoder_event evt);
static void update_scroll(void);

static struct menu main_menu;
static struct menu commands_menu;

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
    const char *evt_names[] = {
        [ENCODER_ROTATE_CW] = "ROTATE_CW",
        [ENCODER_ROTATE_CCW] = "ROTATE_CCW",
        [ENCODER_DOUBLE_PRESS] = "DOUBLE_PRESS",
        [ENCODER_PRESS_ROTATE_CW] = "PRESS_ROTATE_CW",
        [ENCODER_PRESS_ROTATE_CCW] = "PRESS_ROTATE_CCW",
    };
    LOG_INF("Menu EVT: %s (sel=%d confirm=%d active_draw=%d)",
            evt_names[evt], state.selected, state.showing_confirmation, state.active_draw_fn != NULL);

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
            LOG_INF("Menu: sel++ -> %d", state.selected);
        } else {
            LOG_INF("Menu: sel at bottom, ignored");
        }
        break;
    case ENCODER_ROTATE_CCW:
        if (state.selected > 0) {
            state.selected--;
            update_scroll();
            LOG_INF("Menu: sel-- -> %d", state.selected);
        } else {
            LOG_INF("Menu: sel at top, ignored");
        }
        break;
    case ENCODER_DOUBLE_PRESS: {
        struct menu_item *item = &state.current->items[state.selected];
        LOG_INF("Menu: DOUBLE_PRESS on '%s' (draw=%d sub=%d act=%d conf=%d)",
                item->label, item->draw_fn != NULL, item->submenu != NULL,
                item->action_fn != NULL, item->needs_confirmation);
        if (item->draw_fn) {
            state.active_draw_fn = item->draw_fn;
            state.active_title = item->label;
            clear_display();
            LOG_INF("Menu: entered draw_fn '%s'", item->label);
        } else if (item->submenu) {
            state.current = item->submenu;
            state.selected = 0;
            state.scroll_offset = 0;
            LOG_INF("Menu: entered submenu '%s'", item->label);
        } else if (item->action_fn) {
            if (item->needs_confirmation) {
                state.showing_confirmation = true;
                state.confirming_item = item;
                LOG_INF("Menu: showing confirmation for '%s'", item->label);
            } else {
                item->action_fn();
                LOG_INF("Menu: executed action '%s'", item->label);
            }
}
        break;
    }
    case ENCODER_PRESS_ROTATE_CW: {
        if (state.current->item_count == 0 || state.selected >= state.current->item_count) {
            break;
        }
        struct menu_item *item = &state.current->items[state.selected];
        LOG_INF("Menu: PRESS_ROTATE_CW on '%s' (draw=%d sub=%d)",
                item->label, item->draw_fn != NULL, item->submenu != NULL);
        if (item->draw_fn) {
            state.active_draw_fn = item->draw_fn;
            state.active_title = item->label;
            clear_display();
            LOG_INF("Menu: entered draw_fn '%s'", item->label);
        } else if (item->submenu) {
            state.current = item->submenu;
            state.selected = 0;
            state.scroll_offset = 0;
            clear_display();
            LOG_INF("Menu: entered submenu '%s'", item->label);
        }
        break;
    }
    case ENCODER_PRESS_ROTATE_CCW:
        LOG_INF("Menu: PRESS_ROTATE_CCW (active_draw=%d parent=%d)",
                state.active_draw_fn != NULL, state.current->parent != NULL);
        if (state.active_draw_fn) {
            state.active_draw_fn = NULL;
            state.active_title = NULL;
            clear_display();
            LOG_INF("Menu: exited draw_fn, back to menu");
        } else if (state.current->parent) {
            state.current = state.current->parent;
            state.selected = 0;
            state.scroll_offset = 0;
            LOG_INF("Menu: exited to parent menu");
        } else {
            LOG_INF("Menu: PRESS_ROTATE_CCW ignored (no parent, no active_draw)");
        }
        break;
    }
}

void menu_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("Menu: Thread Started");
    k_poll_event_init(&events[0], K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                      K_POLL_MODE_NOTIFY_ONLY, &encoder_msgq);
    k_poll_event_init(&events[1], K_POLL_TYPE_SEM_AVAILABLE,
                      K_POLL_MODE_NOTIFY_ONLY, &menu_data_sem);

while (1) {
        LOG_DBG("Waiting for menu events");
        k_poll(events, 2, K_FOREVER);
        LOG_DBG("Updating Menu");

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
    LOG_DBG("Starting menu thread");
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

static void cmd_set_manual_mode(void)
{
    LOG_INF("MAVLink: Setting Manual Mode");
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(1, 69, &msg, 1,1, MAV_CMD_DO_SET_MODE , MAV_MODE_MANUAL_DISARMED, 0, 0, 0.0f ,0.0f, 0.0f, 0.0f, 0.0f  );
    
    mavwrap_send_message(__mav_devices.mav_lora, &msg);
}

static void cmd_do_wiggle_servo(void)
{
    LOG_INF("MAVLink: Setting Servo");

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(1, 69, &msg, 1,1, MAV_CMD_DO_SET_SERVO , 0 , 30, 0, 0.0f ,0.0f, 0.0f, 0.0f, 0.0f  );
    
    mavwrap_send_message(__mav_devices.mav_lora, &msg);
}

static void cmd_reboot(void)
{
    LOG_INF("MAVLink: Would send Reboot");
}

static struct menu_item commands_items[] = {
    {"Arm/Disarm", NULL, cmd_arm_disarm, NULL, true},
    {"Set Manual Mode", NULL, cmd_set_manual_mode, NULL, false},
    {"Wiggle Servo", NULL, cmd_do_wiggle_servo, NULL, true},
    {"CAGA", NULL, cmd_reboot, NULL, true},
};


static struct menu_item main_items[] = {
    {"COMMS",    draw_comms_screen,  NULL, NULL, false},
    {"GPS",      draw_gps_screen,    NULL, NULL, false},
    {"Status",   draw_status_screen, NULL, NULL, false},
    {"Commands", NULL,               NULL, &commands_menu, false},
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

int menu_init(const struct device *mav_lora,const  struct device *mav_udp)
{
    __mav_devices.mav_lora = mav_lora;
    __mav_devices.mav_udp = mav_udp;


    encoder_input_init();

    display_init();

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