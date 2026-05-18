#ifndef MENU_H
#define MENU_H

#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define MENU_VISIBLE_ROWS 7
#define MENU_CURSOR_ROW 3

struct menu_data;

typedef void (*menu_draw_fn)(void);
typedef void (*menu_action_fn)(void);

struct menu_item {
    const char *label;
    menu_draw_fn draw_fn;
    menu_action_fn action_fn;
    struct menu *submenu;
    bool needs_confirmation;
};

struct menu {
    const char *title;
    struct menu_item *items;
    size_t item_count;
    struct menu *parent;
};

struct menu_state {
    struct menu *current;
    size_t selected;
    size_t scroll_offset;
    bool showing_confirmation;
    struct menu_item *confirming_item;
};

struct menu_display_data {
    struct {
        atomic_t tx_count;
        atomic_t rx_count;
        atomic_t errors;
        atomic_t last_rssi;
        atomic_t last_snr;
    } lora;

    struct {
        atomic_t tx_count;
        atomic_t rx_count;
        atomic_t errors;
    } udp;

    struct {
        bool valid;
        uint8_t satellites;
        uint8_t fix_type;
        int32_t latitude;
        int32_t longitude;
        int32_t altitude_mm;
    } gps;

    struct {
        const char *flight_mode;
        uint16_t battery_mv;
        bool armed;
        uint8_t state;
    } asterics;
};

extern struct menu_display_data menu_data;
extern struct k_sem menu_data_sem;

int menu_init(void);

void menu_start(void);

void menu_update_lora_stats(uint32_t tx, uint32_t rx, int16_t rssi, int8_t snr);

void menu_update_udp_stats(uint32_t tx, uint32_t rx);

void menu_update_gps(bool valid, uint8_t sats, uint8_t fix, int32_t lat, int32_t lon, int32_t alt);

void menu_update_asterics(const char *mode, uint16_t battery_mv, bool armed, uint8_t state);

#endif /* MENU_H */