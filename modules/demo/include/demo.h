#ifndef DEMO_H
#define DEMO_H
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#if defined(CONFIG_LIB_MENU)
#include "menu.h"
#endif

enum demo_neopixel_mode { DEMO_NEOPIXEL_OFF, DEMO_NEOPIXEL_BOUNCE, DEMO_NEOPIXEL_SPIN, DEMO_NEOPIXEL_BLINK };
//enum demo_servo_mode    { DEMO_SERVO_OFF, DEMO_SERVO_WIGGLE, DEMO_SERVO_SWEEP, DEMO_SERVO_HELLO };
int  demo_init(void);
bool demo_active(void);
void demo_neopixel_start(enum demo_neopixel_mode mode);
void demo_neopixel_stop(void);
//void demo_servo_start(enum demo_servo_mode mode);
//void demo_servo_stop(void);
void demo_off(void);
int  demo_mavlink_init(const struct device *mav_dev);

/* LoRa observations reported over Ethernet; the radio payload remains PING/PONG. */
enum demo_lora_result {
    DEMO_LORA_PONG = 1,
    DEMO_LORA_TIMEOUT = 2,
    DEMO_LORA_UNEXPECTED = 3,
    DEMO_LORA_ERROR = 4,
    DEMO_LORA_UNAVAILABLE = 5,
};
struct demo_lora_stats {
    uint32_t sequence, tx, rx, pong, timeouts, errors;
    int32_t result, rssi, snr, rtt_ms, age_ms;
};
void demo_mavlink_publish_lora(const struct demo_lora_stats *stats);

#if defined(CONFIG_LIB_MENU)
extern struct menu demo_menu;   /* hook for the main menu */
#endif

#endif /* DEMO_H */
