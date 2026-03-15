#pragma once
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>

void ws2812_bb_init(const struct gpio_dt_spec *din_pin);
int  ws2812_bb_update(const struct gpio_dt_spec *din_pin,
                      struct led_rgb *pixels, size_t num_pixels);