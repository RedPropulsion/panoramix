#define DT_DRV_COMPAT worldsemi_ws2812

#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

#define CPU_CLOCK 550000000ULL

//struct to save dt data
struct ws2812_data {
    const struct gpio_dt_spec gpio;
    uint32_t chain_length;
    uint32_t t1h, t1l, t0h, t0l, reset_delay;
};

static inline void delay_ns(uint32_t cycles) {
    uint32_t start = k_cycle_get_32();
    while (k_cycle_get_32() - start < cycles) {
        // wait
    }
}

static void latch_reset(const struct ws2812_data *config){

        gpio_pin_set_dt(&config->gpio, 0);
        delay_ns(config->reset_delay);
}

static int ws2812_update_rgb(const struct device *dev, struct led_rgb *pixels, size_t num_pixels) {
    const struct ws2812_data *config = dev->config;

    if (num_pixels > config->chain_length) {
        return -EINVAL;
    }

    // calculates the number of clock cycles for every logical voltage level
    uint32_t cyc_t1h = (config->t1h * CPU_CLOCK) / 1000000000ULL;
    uint32_t cyc_t1l = (config->t1l * CPU_CLOCK) / 1000000000ULL;
    uint32_t cyc_t0h = (config->t0h * CPU_CLOCK) / 1000000000ULL;
    uint32_t cyc_t0l = (config->t0l * CPU_CLOCK) / 1000000000ULL;

    for (size_t i = 0; i < num_pixels; i++) {
        uint8_t colors[3] = {pixels[i].g, pixels[i].r, pixels[i].b}; //GRB order

        for (int c = 0; c < 3; c++) {
            uint8_t byte = colors[c];
            
            for (int b = 7; b >= 0; b--) {
                if (byte & (1 << b)) {
                    // bit 1
                    gpio_pin_set_dt(&config->gpio, 1);
                    delay_ns(config->t1h);
                    gpio_pin_set_dt(&config->gpio, 0);
                    delay_ns(config->t1l);
                } else {
                    // bit 0
                    gpio_pin_set_dt(&config->gpio, 1);
                    delay_ns(config->t0h);
                    gpio_pin_set_dt(&config->gpio, 0);
                    delay_ns(config->t0l);
                }
            }
        }
    }

    latch_reset(&config);
    return 0;
}

//maps the update_rbg zephyr function to the one defined above
static const struct led_strip_driver_api ws2812_api = {
    .update_rgb = ws2812_update_rgb,
};

static int ws2812_init(const struct device *dev) {
    const struct ws2812_data *config = dev->config;

    if (!gpio_is_ready_dt(&config->gpio)) {
        return -ENODEV;
    }

    return gpio_pin_configure_dt(&config->gpio, GPIO_OUTPUT_INACTIVE);
}

#define WS2812_INIT(i) \
    static const struct ws2812_data ws2812_data_##i = {  /* ## token pasting operator, allocates variables with different names as i varies in the for loop */ \
        .gpio = GPIO_DT_SPEC_INST_GET(i, gpios), \
        .chain_length = DT_INST_PROP(i, chain_length), \
        .t1h = DT_INST_PROP_OR(i, delay_t1h, 940), \
        .t1l = DT_INST_PROP_OR(i, delay_t1l, 310), \
        .t0h = DT_INST_PROP_OR(i, delay_t0h, 310), \
        .t0l = DT_INST_PROP_OR(i, delay_t0l, 940), \
        .reset_delay = DT_INST_PROP_OR(i, reset_delay, 50000), \
    }; \
    DEVICE_DT_INST_DEFINE(i, ws2812_init, NULL, NULL, \
                          &ws2812_data_##i, POST_KERNEL, \
                          CONFIG_LED_STRIP_INIT_PRIORITY, \
                          &ws2812_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_INIT)

