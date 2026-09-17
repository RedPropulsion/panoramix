#ifndef DEMO_H
#define DEMO_H
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#if defined(CONFIG_LIB_MENU)
#include "menu.h"
#endif

enum demo_neopixel_mode { DEMO_NEOPIXEL_OFF, DEMO_NEOPIXEL_BOUNCE, DEMO_NEOPIXEL_SPIN, DEMO_NEOPIXEL_BLINK };
enum demo_servo_mode    { DEMO_SERVO_OFF, DEMO_SERVO_WIGGLE, DEMO_SERVO_SWEEP, DEMO_SERVO_HELLO };
int  demo_init(void);
bool demo_active(void);
void demo_neopixel_start(enum demo_neopixel_mode mode);
void demo_neopixel_stop(void);
void demo_servo_start(enum demo_servo_mode mode);
void demo_servo_stop(void);
void demo_off(void);
int  demo_mavlink_init(const struct device *mav_dev);

#if defined(CONFIG_LIB_MENU)
extern struct menu demo_menu;   /* hook for the main menu */
#endif

#endif /* DEMO_H */