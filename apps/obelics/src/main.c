#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/display/cfb.h>
#include <cfb_font_templeos.h>
#include <gnss_u_blox_m10.h>
#include <display.h>



/* ------------------------------------------------------------------ *
 * MavWrap
 * ------------------------------------------------------------------ */






/* ------------------------------------------------------------------ *
 * File logger
 * ------------------------------------------------------------------ */


/* ------------------------------------------------------------------ *
 * LoRa
 * ------------------------------------------------------------------ */
// #define LORA_NODE DT_NODELABEL(lora_sx1261)
// static const struct device *lora_dev = DEVICE_DT_GET(LORA_NODE);
// static struct lora_modem_config lora_tx_config;

/* ------------------------------------------------------------------ *
 * Neopixel
 * ------------------------------------------------------------------ */
#define STRIP_NODE  DT_NODELABEL(led_strip)
#define NUM_LEDS    DT_PROP(STRIP_NODE, chain_length)

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec neopixel_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_en), gpios);


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
// static struct k_work button_work;
// static uint8_t demo = 0;

// void sound_finished_cb(void) {
//     LOG_INF("Sound playback finished");
// }

// static void button_work_handler(struct k_work *work)
// {
//     ARG_UNUSED(work);
//     LOG_INF("Button work: toggling sound demo");
//     switch (demo++ % 4) {
//     case 0: play_sound(success_sound, success_sound_len, sound_finished_cb); break;
//     case 1: play_sound(alert_sound, alert_sound_len, sound_finished_cb); break;
//     case 2: play_sound(acknowledge_sound, acknowledge_sound_len, sound_finished_cb); break;
//     case 3: play_sound(error_sound, error_sound_len, sound_finished_cb); break;
//     }
// }

/* ------------------------------------------------------------------ *
 * LED blink timers
 * ------------------------------------------------------------------ */


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

    
    // udp_client_init();
    
    

    // if (!device_is_ready(i2c_dev)) {
    //     LOG_ERR("I2C device not ready");
    //     return 0;
    // }
    // i2c_scan_bus(i2c_dev);


/* Oled Display */
    

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

    
    
    // /* Initialize LoRa device */
    // if (!device_is_ready(lora_dev)) {
    //     LOG_ERR("LoRa device not ready");
    // } else {
    //     lora_tx_config.frequency = 868000000;
    //     lora_tx_config.bandwidth = BW_125_KHZ;
    //     lora_tx_config.datarate = SF_7;
    //     lora_tx_config.coding_rate = CR_4_5;
    //     lora_tx_config.preamble_len = 12;
    //     lora_tx_config.tx_power = 4;
    //     lora_tx_config.tx = true;
    //     lora_tx_config.iq_inverted = false;
    //     lora_tx_config.public_network = false;

    //     ret = lora_config(lora_dev, &lora_tx_config);
    //     if (ret < 0) {
    //         LOG_ERR("LoRa config failed: %d", ret);
    //     } else {
    //         LOG_INF("LoRa initialized: 868 MHz, SF7, 4 dBm");
    //     }
    // }

    
    // k_work_init(&button_work, button_work_handler);
    // sound_init(&buzzer);

    
    /* LEDs */
    


    /* B1 User button */
    if (!gpio_is_ready_dt(&user_btn)) {
        LOG_ERR("User button not ready");
        return 0;
    }
    gpio_pin_configure_dt(&user_btn, GPIO_INPUT);

    ret = gpio_pin_interrupt_configure_dt(&user_btn, GPIO_INT_EDGE_FALLING);

    // gpio_init_callback(&btn_cb_data, button_handler, BIT(user_btn.pin));
    // ret = gpio_add_callback(user_btn.port, &btn_cb_data);

    // display_update_row(3, "Buttons OK");

    
    // udp_client_init();
    display_update_row(5, "UDP Ok");


    struct gps_position pos;
    if (gps_get_latest(&pos) == 0) {
        LOG_INF("GPS: %s data \n\t\t %d sats\n\t\t %d fix type  ",pos.valid ? "Valid" : "Invalid", pos.satellites, pos.fix_type);
        display_update_row(6, "GPS: %d sats", pos.satellites);
    } else {
        display_update_row(6, "GPS: No fix");
    }



    bool toggle = false;
    // uint8_t tx_buf[] = "Hello from Obelics!";
    

    // display_string("Main loop");


    // display_clear_text();

    LOG_INF("Menu: init ");
    menu_init(mavlink_lora, mavlink_udp, &buzzer);

    

    LOG_INF("Starting main loop...");
    // int lora_counter = 0;
    // int row=0;
    while (1) {

        // display_string("Running main loop...");
        /* LoRa TX every 5 seconds */
        // if (device_is_ready(lora_dev)) {
        //     int err = lora_send(lora_dev, tx_buf, sizeof(tx_buf));
        //     if (err == 0) {
        //         LOG_DBG("LoRa TX #%d: %d bytes", lora_counter++, (int)sizeof(tx_buf));
        //     } else {
        //         LOG_ERR("LoRa TX failed: %d", err);
        //     }
        // }

        
        // if (toggle) {
        //     pixels[0].r = 128;
        //     pixels[3].r = 128;
        //     pixels[3].b = 128;
        // } else {
        //     pixels[1].b = 128;
        //     pixels[2].g = 128;
        // }
        // toggle = !toggle;        

        

        // int err = lora_recv(lora_dev, buf, sizeof(buf), K_SECONDS(2), &RSSI, &SNR);
        // if (err == -EAGAIN) {
        //     LOG_DBG("No LoRa RX data yet");
        //     // display_update_row(7, "No LoRa RX data");
        // } else if (err < 0) {
        //   menu_update_lora_stats(lora_counter, err, 0, 0);
        //     LOG_ERR("LoRa RX failed: %d", err);
        //     // display_update_row(7, "LoRa RX failed: %d", err);
        // } else {
        //   menu_update_lora_stats(lora_counter, err, (int16_t)RSSI, (int8_t)SNR);
        //     LOG_INF("LoRa RX: %d bytes: %s", err, buf);
        //     LOG_INF("RSSI: %d, SNR: %d", RSSI, SNR);
        //     // display_update_row(7, "RSSI:%d SNR:%d", RSSI, SNR);
        // }
        // row = (row + 1) % 8; // cycle through display rows for updates


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
        k_sleep(K_SECONDS(1));
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
