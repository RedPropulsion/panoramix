#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/drivers/display.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/display/cfb.h>
#include "sound.h"
#include "gnss_u_blox_m10.h"
#include "display.h"
#include "menu.h"
#include <cfb_font_templeos.h>
#include <zephyr/drivers/i2c.h>
#include "file_logger.h"
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/drivers/uart.h>
#include "demo.h"


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ *
 * File logger
 * ------------------------------------------------------------------ */

static struct file_logger_file log_file;

static void init_storage(void)
{
    int ret = file_logger_init();
    if (ret < 0) {
        LOG_ERR("Failed to init file logger: %d", ret);
        return;
    }

    ret = file_logger_open("/SD:/packets.log", FS_O_CREATE | FS_O_READ | FS_O_WRITE | FS_O_APPEND, &log_file);
    if (ret < 0) {
        LOG_ERR("Failed to open log file: %d", ret);
        return;
    }

    LOG_INF("File logger initialized and log file opened");
}

/* ------------------------------------------------------------------ *
 * LoRa
 * ------------------------------------------------------------------ */
#define LORA_NODE DT_NODELABEL(lora_sx1261)
static const struct device *lora_dev = DEVICE_DT_GET(LORA_NODE);
static struct lora_modem_config lora_cfg = {
    .frequency = 868000000,
    .bandwidth = BW_250_KHZ,
    .datarate = SF_8,
    .coding_rate = CR_4_5,
    .preamble_len = 12,
    .tx_power = 14,
    .tx = true,
    .iq_inverted = false,
    .public_network = false,
    .packet_crc_disable = false,
};

/* Keep the existing WL55 firmware and four-byte PING/PONG protocol. */
static struct demo_lora_stats lora_stats = {
    .result = DEMO_LORA_UNAVAILABLE, .rtt_ms = -1, .age_ms = -1,
};
static int64_t lora_last_pong_ms = -1;

static void lora_test_once(void)
{
    uint8_t tx_buf[] = "PING";
    uint8_t rx_buf[256];
    int16_t rssi;
    int8_t snr;
    int ret;
    int64_t sent_at;

    lora_stats.result = DEMO_LORA_ERROR;
    lora_stats.rtt_ms = -1;
    lora_cfg.tx = true;
    ret = lora_config(lora_dev, &lora_cfg);
    if (ret < 0) {
        LOG_ERR("LoRa TX config failed: %d", ret);
        goto update_stats;
    }
    sent_at = k_uptime_get();
    ret = lora_send(lora_dev, tx_buf, sizeof(tx_buf) - 1);
    if (ret < 0) {
        LOG_ERR("LoRa TX failed: %d", ret);
        goto update_stats;
    }
    lora_stats.tx++;
    lora_cfg.tx = false;
    ret = lora_config(lora_dev, &lora_cfg);
    if (ret < 0) {
        LOG_ERR("LoRa RX config failed: %d", ret);
        goto update_stats;
    }
    ret = lora_recv(lora_dev, rx_buf, sizeof(rx_buf) - 1,
                    K_SECONDS(2), &rssi, &snr);
    LOG_INF("LoRa TX #%u: PING", (unsigned int)lora_stats.tx);
    if (ret == -EAGAIN || ret == -ETIMEDOUT) {
        lora_stats.timeouts++;
        lora_stats.result = DEMO_LORA_TIMEOUT;
        LOG_INF("LoRa RX timeout: no reply");
    } else if (ret < 0) {
        LOG_ERR("LoRa RX failed: %d", ret);
    } else {
        lora_stats.rx++;
        if (ret == 4 && memcmp(rx_buf, "PONG", 4) == 0) {
            lora_last_pong_ms = k_uptime_get();
            lora_stats.pong++;
            lora_stats.result = DEMO_LORA_PONG;
            lora_stats.rssi = rssi;
            lora_stats.snr = snr;
            lora_stats.rtt_ms = (int32_t)(lora_last_pong_ms - sent_at);
            LOG_INF("LoRa PING/PONG OK: RSSI %d dBm, SNR %d dB, RTT %d ms",
                    (int)rssi, (int)snr, (int)lora_stats.rtt_ms);
        } else {
            lora_stats.result = DEMO_LORA_UNEXPECTED;
            LOG_WRN("LoRa received %d bytes, expected four bytes PONG", ret);
        }
    }

update_stats:
    if (lora_stats.result == DEMO_LORA_ERROR) {
        lora_stats.errors++;
    }
    /* Keep OLED packet counters; signal metrics refer only to valid PONGs. */
    menu_update_lora_stats(lora_stats.tx, lora_stats.rx,
                          lora_stats.rssi, lora_stats.snr);
}

/* The Ethernet peer may power up after ObelICS. Only start once it is ready.
 * Called from main after demo_init(), never from the LED/radio workqueue.
 */
static void mavlink_try_start(void)
{
    static bool started;
    static bool waiting_logged;
    const struct device *mav_dev =
        DEVICE_DT_GET(DT_NODELABEL(mavlink_idefix));
    const struct device *eth_dev =
        DEVICE_DT_GET(DT_PHANDLE(DT_NODELABEL(mavlink_idefix), transport));
    struct net_if *iface;
    int ret;

    /* Do not allocate another UDP context after a successful start. */
    if (started) {
        return;
    }
    if (!device_is_ready(mav_dev)) {
        LOG_ERR("MAVLink device not ready; cannot start transport");
        return;
    }
    iface = net_if_lookup_by_dev(eth_dev);
    if (!iface) {
        LOG_ERR("MAVLink Ethernet interface not found");
        return;
    }
    /* Skip the wrapper's blocking network wait while Ethernet is down. */
    if (!net_if_is_up(iface)) {
        if (!waiting_logged) {
            LOG_INF("MAVLink waiting for Ethernet; LoRa remains active");
            waiting_logged = true;
        }
        return;
    }
    waiting_logged = false;
    ret = demo_mavlink_init(mav_dev);
    if (ret < 0) {
        LOG_WRN("MAVLink start failed: %d; will retry next cycle", ret);
        return;
    }
    started = true;
    LOG_INF("MAVLink UDP ready: 192.168.10.2:14550 -> 192.168.10.1:14551");
}

/* ------------------------------------------------------------------ *
 * Neopixel
 * ------------------------------------------------------------------ */
#define STRIP_NODE  DT_NODELABEL(led_strip)

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec neopixel_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_en), gpios);

/* ------------------------------------------------------------------ *
 * Servo
 * ------------------------------------------------------------------ */

#if 0 /* Servo devices disabled for the LED/LoRa demo. */
 const struct device *yaw_servo = DEVICE_DT_GET(DT_NODELABEL(yaw_servo));
 const struct device *pitch_servo = DEVICE_DT_GET(DT_NODELABEL(pitch_servo));
#endif

/* ------------------------------------------------------------------ *
 * Buzzer
 * ------------------------------------------------------------------ */

static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_NODELABEL(buzzer));

/* ------------------------------------------------------------------ *
 * Button
 * ------------------------------------------------------------------ */
static const struct gpio_dt_spec user_btn =
    GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);

static struct gpio_callback btn_cb_data;
static struct k_work button_work;
static uint8_t demo = 0;

void sound_finished_cb(void) {
    LOG_INF("Sound playback finished");
}

static void button_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_INF("Button work: toggling sound demo");
    switch (demo++ % 4) {
    case 0: play_sound(success_sound, success_sound_len, sound_finished_cb); break;
    case 1: play_sound(alert_sound, alert_sound_len, sound_finished_cb); break;
    case 2: play_sound(acknowledge_sound, acknowledge_sound_len, sound_finished_cb); break;
    case 3: play_sound(error_sound, error_sound_len, sound_finished_cb); break;
    }
}

void button_handler(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins)
{
    k_work_submit(&button_work);
}

/* ------------------------------------------------------------------ *
 * LED blink timers
 * ------------------------------------------------------------------ */
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

void led_timer_handler(struct k_timer *timer_id) {
    struct led_data *led = CONTAINER_OF(timer_id, struct led_data, timer);
    gpio_pin_toggle_dt(&led->gpio);
    k_timer_start(&led->timer, K_MSEC(intervals[led->index]), K_NO_WAIT);
}

/* ------------------------------------------------------------------ *
 * Oled Display
 * ------------------------------------------------------------------ */

const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));



/* ------------------------------------------------------------------ *
 * GPS
 * ------------------------------------------------------------------ */

 

/* ------------------------------------------------------------------ *
 * Main
 * ------------------------------------------------------------------ */
int main(void)
{   
     LOG_INF("Starting main()");
    int ret;

    #if 0 /* Preserve the startup test without moving the damaged servos. */
    int32_t angle_mdeg = 0;

    k_sleep(K_MSEC(500));

    LOG_INF("YAW - PING");
    ret = servo_ping(yaw_servo);
    LOG_INF("yaw ping ret=%d", ret);

    k_sleep(K_MSEC(200));

    LOG_INF("PITCH - PING");
    ret = servo_ping(pitch_servo);
    LOG_INF("pitch ping ret=%d", ret);

    k_sleep(K_MSEC(200));

    LOG_INF("YAW - SET - 0");
    ret = servo_set_position(yaw_servo, 0);
    LOG_INF("yaw set ret=%d", ret);

    k_msleep(50);

    LOG_INF("YAW - GET");
    ret = servo_get_position(yaw_servo, &angle_mdeg);
    LOG_INF("yaw get ret=%d pos=%d mdeg", ret, angle_mdeg);

    k_sleep(K_MSEC(500));

    LOG_INF("PITCH - SET - 210");
    ret = servo_set_position(pitch_servo, 210 * 1000);
    LOG_INF("pitch set ret=%d", ret);

    k_msleep(50);

    LOG_INF("PITCH - GET");
    ret = servo_get_position(pitch_servo, &angle_mdeg);
    LOG_INF("pitch get ret=%d pos=%d mdeg", ret, angle_mdeg);

    k_sleep(K_MSEC(500));

    LOG_INF("YAW - SET - 90");
    ret = servo_set_position(yaw_servo, 90 * 1000);
    LOG_INF("yaw set ret=%d", ret);

    k_msleep(50);

    LOG_INF("YAW - GET");
    ret = servo_get_position(yaw_servo, &angle_mdeg);
    LOG_INF("yaw get ret=%d pos=%d mdeg", ret, angle_mdeg);

    uint8_t pitch_status = 0;
    LOG_INF("PITCH - GET - STATUS");
    ret = servo_get_status(pitch_servo, &pitch_status);
    LOG_INF("pitch status ret=%d status=0x%02X", ret, pitch_status);

    k_msleep(500);

    LOG_INF("PITCH - SET - 260");
    ret = servo_set_position(pitch_servo, 260 * 1000);
    LOG_INF("pitch set ret=%d", ret);

    k_msleep(50);

    LOG_INF("PITCH - GET");
    ret = servo_get_position(pitch_servo, &angle_mdeg);
    LOG_INF("pitch get ret=%d pos=%d mdeg", ret, angle_mdeg);
    

    #endif

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }
    // i2c_scan_bus(i2c_dev);


/* Oled Display */
    ret = display_init();
    if (ret < 0) {
        LOG_ERR("Display init failed: %d", ret);
    }
    display_string("test");
    display_update_row(0, "Starting");

    // cfb_framebuffer_finalize(disp);


    // uint8_t width, height;
    // uint8_t num_fonts = cfb_get_numof_fonts(disp);

    // // Loop through available fonts to get their dimensions
    // for (uint8_t i = 0; i < num_fonts; i++) {
    //     cfb_get_font_size(disp, i, &width, &height);
    //     // printk("Font Index %d: %dx%d pixels\n", i, width, height);
    //     cfb_framebuffer_set_font(disp, i);
    //     display_string("Font %d: %dx%d", i, width, height);
    //     k_sleep(K_SECONDS(2));
    // }

    // cfb_framebuffer_set_font(disp, 0);

    #ifdef CONFIG_FAT_FILESYSTEM_ELM
    /* Initialize file logger */
    init_storage();

    /* Write test message to log file */
    ret = file_logger_write_str(&log_file, "Start\n");
    if (ret < 0) {
        LOG_ERR("Failed to write to log file: %d", ret);
    } else {
        file_logger_flush(&log_file);
        LOG_DBG("Wrote test to log file"); 
    }

    /* Read back to verify */
    file_logger_seek(&log_file, 0, FS_SEEK_SET);
    char read_buff[64];
    int len = file_logger_read_str(&log_file, read_buff, sizeof(read_buff));
    LOG_INF("Log file content (%d bytes): %s", len, read_buff);

    /* Clear log file content by removing and recreating it */
    file_logger_close(&log_file);
    file_logger_remove("/SD:/packets.log");
    file_logger_open("/SD:/packets.log", FS_O_CREATE | FS_O_READ | FS_O_WRITE | FS_O_APPEND, &log_file);
    #endif

    /* A ready device is not yet proof of a working radio link. */
    bool lora_ok = false;
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa device not ready");
        display_update_row(1, "LoRa not ready");
    } else {
        ret = lora_config(lora_dev, &lora_cfg);
        if (ret < 0) {
            LOG_ERR("LoRa config failed: %d", ret);
            display_update_row(1, "LoRa cfg error");
        } else {
            lora_ok = true;
            LOG_INF("LoRa configured: 868 MHz, BW250, SF8, CR4/5");
            display_update_row(1, "LoRa configured");
        }
    }

    /* Neopixel enable */
    gpio_pin_configure_dt(&neopixel_en, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&neopixel_en, 1);

    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }


    if (!device_is_ready(buzzer.dev)) {
        LOG_ERR("Buzzer PWM not ready");
        return -ENODEV;
    }
    k_work_init(&button_work, button_work_handler);
    LOG_INF("Buzzer pointer %p", (void *)&buzzer);
    sound_init(&buzzer);

    
    /* LEDs */
    for (int i = 0; i < ARRAY_SIZE(leds); i++) {
        if (!gpio_is_ready_dt(&leds[i].gpio)) {
            LOG_ERR("LED %d not ready", i);
            return 0;
        }
        gpio_pin_configure_dt(&leds[i].gpio, GPIO_OUTPUT_ACTIVE);
        k_timer_init(&leds[i].timer, led_timer_handler, NULL);
        k_timer_start(&leds[i].timer, K_MSEC(intervals[i]), K_NO_WAIT);
    }

    display_update_row(2, "LED,Sound OK");


    /* B1 User button */
    if (!gpio_is_ready_dt(&user_btn)) {
        LOG_ERR("User button not ready");
        return 0;
    }
    gpio_pin_configure_dt(&user_btn, GPIO_INPUT);

    ret = gpio_pin_interrupt_configure_dt(&user_btn, GPIO_INT_EDGE_FALLING);

    gpio_init_callback(&btn_cb_data, button_handler, BIT(user_btn.pin));
    ret = gpio_add_callback(user_btn.port, &btn_cb_data);

    display_update_row(3, "Buttons OK");

    /* MAVLink UDP starts after demo_init(), below. */


    struct gps_position pos;
    if (gps_get_latest(&pos) == 0) {
        LOG_INF("GPS: %s data \n\t\t %d sats\n\t\t %d fix type  ",pos.valid ? "Valid" : "Invalid", pos.satellites, pos.fix_type);
        display_update_row(6, "GPS: %d sats", pos.satellites);
    } else {
        display_update_row(6, "GPS: No fix");
    }



    

    LOG_INF("Starting main loop...");
    // display_string("Main loop");
    display_update_row(5, "Init complete");
    k_sleep(K_MSEC(1000));

    display_clear_text();

    menu_init();

    menu_start();

    ret = demo_init();
    if (ret < 0) {
        LOG_ERR("Demo init failed: %d", ret);
        return ret;
    }

    while (1) {
        int64_t cycle_start = k_uptime_get();

        /* demo_init() above must finish before enabling command callbacks. */
        mavlink_try_start();

        lora_stats.sequence++;
        if (lora_ok) {
            lora_test_once();
        }
        lora_stats.age_ms = lora_last_pong_ms < 0 ? -1 :
            (int32_t)MIN(k_uptime_get() - lora_last_pong_ms, INT32_MAX);
        demo_mavlink_publish_lora(&lora_stats);

        /* The demo module exclusively controls the LED strip, including OFF. */

        // LOG_DBG("GPS: %02d/%02d/%04d %02d:%02d:%02d.%03u | "
        //                             "lat=%d, lon=%d, alt=%dmm | "
        //                             "sats=%d, fix=%d, hdop=%d | "
        //                             "speed=%dmm/s, heading=%d.%05d | "
        //                             "acc: horiz=%umm, vert=%umm",
        //                             pos.day, pos.month, pos.year,
        //                             pos.hour, pos.minute, pos.second, pos.nanosecond / 1000000,
        //                             pos.latitude, pos.longitude, pos.altitude_mm,
        //                             pos.satellites, pos.fix_type, pos.hdop,
        //                             pos.speed_mm_s, pos.heading_1e5 / 100000, pos.heading_1e5 % 100000,
        //                             pos.horiz_acc_mm, pos.vert_acc_mm);

        ret = gps_get_latest(&pos);
        if (ret == 0 && pos.valid) {
          menu_update_gps(true, pos.satellites, pos.fix_type, pos.latitude, pos.longitude, pos.altitude_mm);
            // display_update_row(0, "%d sats fix=%d", pos.satellites, pos.fix_type);
            // display_update_row(1, "lat%d", pos.latitude);
            // display_update_row(2,"lon%d", pos.longitude);
            // display_update_row(3,"alt%d", (int)(pos.altitude_mm/1000));
            // display_update_row(4,"h:%d v:%d", (int)(pos.horiz_acc_mm), (int)(pos.vert_acc_mm));   
            // display_update_row(5, "%02d:%02d:%02d", pos.hour, pos.minute, pos.second);
        } else {
            // display_update_row(0, "GPS: No fix");
        }
        /* Keep at least five seconds between test starts, including errors. */
        int64_t remaining_ms = 5000 - (k_uptime_get() - cycle_start);
        k_msleep(remaining_ms > 0 ? (int32_t)remaining_ms : 1);
    }
    return 0;
}


void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf) {
  const struct gpio_dt_spec error_led =
      GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

  LOG_PANIC();

  while (1) {
    display_string("FATAL ERROR");
    LOG_ERR("I'M PANICKING");
    gpio_pin_toggle_dt(&error_led);
    k_busy_wait(500 * 1000);
  }

  k_fatal_halt(reason);
}
