#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>


#include <display.h>
#include <menu.h>

#include "state_machine.h"
#include "zephyr/smf.h"
#include "state_boot.h"


LOG_MODULE_REGISTER(Boot);

#define STRIP_NODE  DT_NODELABEL(led_strip)
#define NUM_LEDS    DT_PROP(STRIP_NODE, chain_length)

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec neopixel_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_en), gpios);


static const struct device *mavlink_lora =
    DEVICE_DT_GET(DT_NODELABEL(mavlink_lora));

static const struct device *mavlink_udp =
    DEVICE_DT_GET(DT_NODELABEL(mavlink_netif));    

static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_NODELABEL(buzzer));
// const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));

static void init_storage(){
  int ret = file_logger_init();
    if (ret < 0) {
        LOG_ERR("Failed to init file logger: %d", ret);
        return;
    }

    ret = file_logger_open("/SD:/packets.log", FS_O_CREATE | FS_O_READ | FS_O_WRITE | FS_O_APPEND, __sfm_state.log_file);
    if (ret < 0) {
        LOG_ERR("Failed to open log file: %d", ret);
        return;
    }

    LOG_INF("File logger initialized and log file opened");
}


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

volatile uint32_t intervals[] = {500, 500, 500};

void led_timer_handler(struct k_timer *timer_id) {
    struct led_data *led = CONTAINER_OF(timer_id, struct led_data, timer);
    gpio_pin_toggle_dt(&led->gpio);
    k_timer_start(&led->timer, K_MSEC(intervals[led->index]), K_NO_WAIT);
}

void boot_entry(){
    int ret;
    LOG_INF("Booting");

    ret = display_init();
    if (ret < 0) {
        LOG_ERR("Display init failed: %d", ret);
    }
    display_update_row(0, "Starting");


    /* Neopixel enable */
    gpio_pin_configure_dt(&neopixel_en, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&neopixel_en, 1);

    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return;
    }


    if (!device_is_ready(buzzer.dev)) {
        LOG_ERR("Buzzer PWM not ready");
        return;
    }

    for (int i = 0; i < ARRAY_SIZE(leds); i++) {
        if (!gpio_is_ready_dt(&leds[i].gpio)) {
            LOG_ERR("LED %d not ready", i);
            return;
        }
        gpio_pin_configure_dt(&leds[i].gpio, GPIO_OUTPUT_ACTIVE);
        k_timer_init(&leds[i].timer, led_timer_handler, NULL);
        k_timer_start(&leds[i].timer, K_MSEC(intervals[i]), K_NO_WAIT);
    }

    display_update_row(1, "LED,Sound OK");


    #ifdef CONFIG_FAT_FILESYSTEM_ELM
    /* Initialize file logger */
    init_storage();

    /* Write test message to log file */
    ret = file_logger_write_str(__sfm_state.log_file, "Start\n");
    if (ret < 0) {
        LOG_ERR("Failed to write to log file: %d", ret);
    } else {
        file_logger_flush(__sfm_state.log_file);
        LOG_DBG("Wrote test to log file"); 
    }

    /* Read back to verify */
    file_logger_seek(__sfm_state.log_file, 0, FS_SEEK_SET);
    char read_buff[64];
    int len = file_logger_read_str(__sfm_state.log_file, read_buff, sizeof(read_buff));
    LOG_INF("Log file content (%d bytes): %s", len, read_buff);

    /* Clear log file content by removing and recreating it */
    file_logger_close(__sfm_state.log_file);
    file_logger_remove("/SD:/packets.log");
    file_logger_open("/SD:/packets.log", FS_O_CREATE | FS_O_READ | FS_O_WRITE | FS_O_APPEND, __sfm_state.log_file);
    display_update_row(2, "Storage Ok");
    #endif

    display_update_row(3, "MAVLink init");
    mavwrap_start(mavlink_lora, lora_rx_callback, NULL);
    mavwrap_start(mavlink_udp, lora_rx_callback, NULL);
    display_update_row(4, "MAVLink Ok");

    struct led_rgb pixels[NUM_LEDS] = {0};
    memset(pixels, 0, sizeof(pixels));

    
    display_update_row(5, "Init complete");
    k_sleep(K_MSEC(1000));

    display_update_row( 6, "Menu Starting");
    LOG_INF("Menu: Starting");
    menu_start();
};


enum smf_state_result boot_run(){
    smf_set_state(ctx, )
    return SMF_EVENT_HANDLED;
};

void boot_exit(){
    return;
};
