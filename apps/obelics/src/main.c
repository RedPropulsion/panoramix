#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/led_strip.h>

#define STRIP_NODE  DT_NODELABEL(led_strip)
#define NUM_LEDS    DT_PROP(STRIP_NODE, chain_length)

static const struct gpio_dt_spec neopixel_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_en), gpios);

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);

/* --- 1. LED Setup & Timers --- */
struct led_data {
    struct gpio_dt_spec gpio;
    struct k_timer timer;
    int index;
};

static struct led_data leds[] = {
    { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios), .index = 0 },
    { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios), .index = 1 },
    { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios), .index = 2 }
};

volatile int selected_led = 0;
volatile uint32_t intervals[] = {500, 500, 500};

void enc_sw_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    static uint32_t last_time = 0;
    uint32_t now = k_uptime_get_32();
    if (now - last_time < 50) return;
    last_time = now;

    selected_led = (selected_led + 1) % ARRAY_SIZE(leds);
    printk("\n---> Now adjusting LED %d <---\n", selected_led);
}

/* --- 2. Encoder Setup --- */
static const struct gpio_dt_spec enc_a  = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_a), gpios);
static const struct gpio_dt_spec enc_b  = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_b), gpios);
static const struct gpio_dt_spec enc_sw = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_s), gpios);

static struct gpio_callback enc_a_cb_data;
static struct gpio_callback enc_sw_cb_data;

void led_timer_handler(struct k_timer *timer_id) {
    struct led_data *led = CONTAINER_OF(timer_id, struct led_data, timer);
    gpio_pin_toggle_dt(&led->gpio);
    k_timer_start(&led->timer, K_MSEC(intervals[led->index]), K_NO_WAIT);
}

void encoder_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int phase_sw = gpio_pin_get_dt(&enc_sw);
    int step = 10;

    if (!phase_sw) {
        static uint32_t last_time = 0;
        uint32_t now = k_uptime_get_32();
        if (now - last_time < 5) return;
        last_time = now;

        int phase_b = gpio_pin_get_dt(&enc_b);
        if (phase_b) {
            intervals[selected_led] += step;
        } else {
            if (intervals[selected_led] > step)
                intervals[selected_led] -= step;
        }
        // Fixed: format string now matches 4 arguments
        printk("LED sel=%d intervals: %d \t %d \t %d ms\n",
               selected_led, intervals[0], intervals[1], intervals[2]);
    } else {
        enc_sw_handler(dev, cb, pins);
    }
}

/* --- 3. Main Initialization --- */
int main(void) {

    // 1. Enable the neopixel buffer (PE5 HIGH) — MUST happen before led_strip_update_rgb
    if (!gpio_is_ready_dt(&neopixel_en)) {
        printk("Error: Neopixel enable pin not ready\n");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&neopixel_en, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&neopixel_en, 1);
    printk("Neopixel buffer enabled\n");

    // 2. Check strip is ready
    if (!device_is_ready(strip)) {
        printk("LED strip not ready\n");
        return -ENODEV;
    }

    // Initialize LEDs and start their timers
    for (int i = 0; i < ARRAY_SIZE(leds); i++) {
        if (!gpio_is_ready_dt(&leds[i].gpio)) {
            printk("Error: LED %d not ready\n", i);
            return 0;
        }
        gpio_pin_configure_dt(&leds[i].gpio, GPIO_OUTPUT_ACTIVE);
        k_timer_init(&leds[i].timer, led_timer_handler, NULL);
        k_timer_start(&leds[i].timer, K_MSEC(intervals[i]), K_NO_WAIT);
    }

    // Initialize Encoder Pins
    if (!gpio_is_ready_dt(&enc_a) || !gpio_is_ready_dt(&enc_b) || !gpio_is_ready_dt(&enc_sw)) {
        printk("Error: Encoder GPIOs not ready\n");
        return 0;
    }
    gpio_pin_configure_dt(&enc_a, GPIO_INPUT);
    gpio_pin_configure_dt(&enc_b, GPIO_INPUT);
    gpio_pin_configure_dt(&enc_sw, GPIO_INPUT);

    gpio_pin_interrupt_configure_dt(&enc_a, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&enc_a_cb_data, encoder_handler, BIT(enc_a.pin));
    gpio_add_callback(enc_a.port, &enc_a_cb_data);

    gpio_pin_interrupt_configure_dt(&enc_sw, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&enc_sw_cb_data, encoder_handler, BIT(enc_sw.pin));
    gpio_add_callback(enc_sw.port, &enc_sw_cb_data);

    printk("Starting main loop...\n");

    // 3. Declare pixels ONCE outside the loop
    struct led_rgb pixels[NUM_LEDS] = {0};
    bool toggle = false;

    while (1) {
        // Clear all pixels first
        memset(pixels, 0, sizeof(pixels));

        if (toggle) {
            pixels[0].r = 128;  // LED 0 red
        } else {
            pixels[1].g = 128;  // LED 1 green
        }
        toggle = !toggle;

        led_strip_update_rgb(strip, pixels, NUM_LEDS);
        k_sleep(K_SECONDS(1));
    }
    return 0;
}