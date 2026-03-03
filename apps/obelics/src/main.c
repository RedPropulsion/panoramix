#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

/* --- 1. LED Setup & Timers --- */
struct led_data {
    struct gpio_dt_spec gpio;
    struct k_timer timer;
    int index;
};

// Define the 3 LEDs and assign them an index
static struct led_data leds[] = {
    { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios), .index = 0 },
    { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios), .index = 1 },
    { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios), .index = 2 }
};

volatile int selected_led = 0;
volatile uint32_t intervals[] = {500, 500, 500}; // Initial 500ms

// Timer expiration handler: Toggles LED and restarts timer with the latest interval
void led_timer_handler(struct k_timer *timer_id) {
    struct led_data *led = CONTAINER_OF(timer_id, struct led_data, timer);
    gpio_pin_toggle_dt(&led->gpio);
    
    // Restart timer to apply any interval changes immediately
    k_timer_start(&led->timer, K_MSEC(intervals[led->index]), K_NO_WAIT);
}


/* --- 2. Encoder Setup --- */
static const struct gpio_dt_spec enc_a  = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_a), gpios);
static const struct gpio_dt_spec enc_b  = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_b), gpios);
static const struct gpio_dt_spec enc_sw = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_s), gpios);

static struct gpio_callback enc_a_cb_data;
static struct gpio_callback enc_sw_cb_data;

void encoder_a_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    // Software Debounce: Ignore extra triggers within 5ms
    static uint32_t last_time = 0;
    uint32_t now = k_uptime_get_32();
    if (now - last_time < 5) return; 
    last_time = now;

    // Read Phase B to determine direction
    int phase_b = gpio_pin_get_dt(&enc_b); 
    
    if (phase_b) {
        intervals[selected_led] += 50; // Slower
    } else {
        if (intervals[selected_led] > 50) 
            intervals[selected_led] -= 50; // Faster
    }
    printk("LED %d Interval: %d ms\n", selected_led, intervals[selected_led]);
}

void sw_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    // Software Debounce for the switch (50ms)
    static uint32_t last_time = 0;
    uint32_t now = k_uptime_get_32();
    if (now - last_time < 50) return;
    last_time = now;

    selected_led = (selected_led + 1) % ARRAY_SIZE(leds);
    printk("\n---> Now adjusting LED %d <---\n", selected_led);
}


/* --- 3. Main Initialization --- */
int main(void) {
    printk("Starting Rotary Encoder LED Controller...\n");

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

    // Setup Interrupts (Trigger Phase A on rising edge, Switch on press)
    gpio_pin_interrupt_configure_dt(&enc_a, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&enc_a_cb_data, encoder_a_handler, BIT(enc_a.pin));
    gpio_add_callback(enc_a.port, &enc_a_cb_data);

    gpio_pin_interrupt_configure_dt(&enc_sw, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&enc_sw_cb_data, sw_handler, BIT(enc_sw.pin));
    gpio_add_callback(enc_sw.port, &enc_sw_cb_data);

    // Main loop can sleep forever; everything is handled by timers and interrupts
    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}