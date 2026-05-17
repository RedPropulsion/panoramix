#include <zephyr/kernel.h>
#include <zephyr/device.h>
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
#include "udp_client.h"
#include "gnss_u_blox_m10.h"
#include <cfb_font_templeos.h>
#include <zephyr/drivers/i2c.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ------------------------------------------------------------------ *
 * SD Card / FATFS
 * ------------------------------------------------------------------ */
#ifdef CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
#include <string.h>
/* FatFs work area */
FATFS fatfs_fs;
/* mounting info */
static struct fs_mount_t fat_fs_mnt = {
	.type = FS_FATFS,
	.fs_data = &fatfs_fs,
	.mnt_point = "/SD:"
};


static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	
	fs_dir_t_init(&dirp);
	
	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		LOG_WRN("Error opening dir %s [%d]", path, res);
		return res;
	}
	
	LOG_INF("Listing dir %s ...", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);
		
		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}
		
		if (entry.type == FS_DIR_ENTRY_DIR) {
			if(strchr(entry.name, '~') != NULL){
				LOG_INF("[DIR ] %s", entry.name);
			}

		} else {
			if(entry.name[0] != '_'){
				LOG_INF("[FILE] %s (size = %zu)",
					entry.name, entry.size);
			}

		}
	}
	
	/* Verify fs_closedir() */
	fs_closedir(&dirp);
	
	return res;
}

static int fatfs_mount()
{
	/* raw disk i/o */
	do {
		static const char *disk_pdrv = "SD";
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;
		
		if (disk_access_init(disk_pdrv) != 0) {
			LOG_ERR("Storage init ERROR!");
			break;
		}
		
		if (disk_access_ioctl(disk_pdrv,
			DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
				LOG_ERR("Unable to get sector count");
				break;
			}
		LOG_INF("Block count %u", block_count);
		
		if (disk_access_ioctl(disk_pdrv,
			DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
				LOG_ERR("Unable to get sector size");
				break;
			}
		LOG_INF("Sector size %u", block_size);
		
		memory_size_mb = (uint64_t)block_count * block_size;
		LOG_INF("Memory Size(MB) %u", (uint32_t)(memory_size_mb >> 20));
	} while (0);

	int res = fs_mount(&fat_fs_mnt);
	
	if (res == 0) {
		LOG_INF("SD Card mounted.");
		lsdir(fat_fs_mnt.mnt_point);
	} else {
		LOG_WRN("Error mounting disk.\n");
	}	
	return 0;
}
#endif
/* ------------------------------------------------------------------ *
 * LoRa
 * ------------------------------------------------------------------ */
#define LORA_NODE DT_NODELABEL(lora_sx1261)
static const struct device *lora_dev = DEVICE_DT_GET(LORA_NODE);
static struct lora_modem_config lora_tx_config;

/* ------------------------------------------------------------------ *
 * Neopixel
 * ------------------------------------------------------------------ */
#define STRIP_NODE  DT_NODELABEL(led_strip)
#define NUM_LEDS    DT_PROP(STRIP_NODE, chain_length)

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec neopixel_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_en), gpios);

/* ------------------------------------------------------------------ *
 * Button
 * ------------------------------------------------------------------ */
static const struct gpio_dt_spec user_btn =
    GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);

static struct gpio_callback btn_cb_data;

void button_handler(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins)
{
    static uint8_t demo = 0;
    switch (demo++ % 4) {
    case 0: play_sound(success_sound, success_sound_len, NULL); break;
    case 1: play_sound(alert_sound, alert_sound_len, NULL); break;
    case 2: play_sound(acknowledge_sound, acknowledge_sound_len, NULL); break;
    case 3: play_sound(error_sound, error_sound_len, NULL); break;
    }
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
 * Encoder
 * ------------------------------------------------------------------ */
static const struct gpio_dt_spec enc_a  = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_a), gpios);
static const struct gpio_dt_spec enc_b  = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_b), gpios);
static const struct gpio_dt_spec enc_sw = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_s), gpios);

static struct gpio_callback enc_a_cb_data;
static struct gpio_callback enc_sw_cb_data;

void enc_sw_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    static uint32_t last_time = 0;
    uint32_t now = k_uptime_get_32();
    if (now - last_time < 50) return;
    last_time = now;
    selected_led = (selected_led + 1) % ARRAY_SIZE(leds);
    LOG_DBG("Selected LED %d <---", selected_led);
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
        LOG_DBG("LED sel=%d intervals: %d \t %d \t %d ms",
               selected_led, intervals[0], intervals[1], intervals[2]);
    } else {
        enc_sw_handler(dev, cb, pins);
    }
}

/* ------------------------------------------------------------------ *
 * Oled Display
 * ------------------------------------------------------------------ */
// #define DISP_NODE DT_NODELABEL(ssd1309);
// const struct device *disp = DEVICE_DT_GET(DISP_NODE);

static const struct device *disp = DEVICE_DT_GET(DT_NODELABEL(ssd1309));

/*
char buf[64];
snprintf(buf, sizeof(buf), "Error: %d", err);

formats printf-style into buf, then use cfb_print(disp, buf, x, y) to display on the screen.
*/
const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));

// void i2c_scan_bus(const struct device *i2c_dev)
// {
//     uint8_t first = 0x04;
//     uint8_t last = 0x77;

//     LOG_INF("Scanning I2C bus %s...", i2c_dev->name);

//     for (uint8_t addr = first; addr <= last; addr++) {
//         struct i2c_msg msgs[1];
//         uint8_t dummy;

//         msgs[0].buf = &dummy;
//         msgs[0].len = 0U;
//         msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

//         if (i2c_transfer(i2c_dev, &msgs[0], 1, addr) == 0) {
//             LOG_INF("  Found device at 0x%02x", addr);
//         }
//     }

//     LOG_INF("Scan complete");

//     LOG_INF("Testing M10 at 0x21...");
//     uint8_t reg = 0xFD;
//     uint8_t buf[2];
//     struct i2c_msg msgs[2] = {
//         { .buf = &reg, .len = 1, .flags = I2C_MSG_WRITE },
//         { .buf = buf, .len = 2, .flags = I2C_MSG_READ | I2C_MSG_STOP },
//     };
//     int ret = i2c_transfer(i2c_dev, msgs, 2, 0x21);
//     if (ret == 0) {
//         uint16_t avail = (buf[0] << 8) | buf[1];
//         LOG_INF("M10 available bytes: %d", avail);
//     } else {
//         LOG_ERR("M10 I2C read failed: %d", ret);
//     }
// }

char display_buff[64] = {0}; // 128x64/8= 16x8 chars max with 8x8 font, so 64 is a safe buffer size for formatted strings

void display_string(const char *fmt, ...)
{
    char buf[64];
    int ret;
    
    va_list args;
    va_start(args, fmt);
    ret = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // LOG_INF("Formatted string length: %d", ret);
    // LOG_INF("Formatted string: %s", buf);
    
    cfb_framebuffer_clear(disp, false);  // clear buffer only
    ret = cfb_print(disp, buf, 0, 0);          // print at (0,0)
    if (ret < 0) {
        LOG_ERR("Failed to print string: %d", ret);
        return;
    } else {
        // LOG_DBG("String printed to framebuffer with return code: %d", ret);
    }
    cfb_framebuffer_finalize(disp);       // WRITE to display
}

void update_row(uint8_t row, const char *fmt, ...)
{
    if (row >= 8) {
        LOG_ERR("Row %d out of bounds", row);
        return;
    }

    char buf[16]; // 16 chars max for one row with 8x8 font on 128x64 display
    int ret;
    

    va_list args;
    va_start(args, fmt);
    ret = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // LOG_INF("Formatted string length: %d", ret);
    // LOG_INF("Formatted string: %s", buf);

    if (ret >= 16) {
        LOG_WRN("Formatted string truncated to fit row: %s", buf);
    }

    // LOG_DBG("Updating row %d: %s", row, buf);

    memcpy((void *)(display_buff + row * 16), buf, sizeof(buf)); // Copy formatted string into the correct row of display_buff
    // LOG_WRN("DISPLAY_BUFF: %s", display_buff);

    // LOG_INF("Updated display_buff:");
    // for (int i = 0; i < 8; i++) {
    //     LOG_INF(" %d: %s", i, display_buff + i * 16);
    // }
    // OR
    //cfb_print(disp,buf,0,row); // print at (0,row) but we don't save previous screen if we use display_string()
    // idk if we actually need to maintain our own display_buff or if we can just call cfb_print()
    
    // cfb_framebuffer_clear(disp, false);  // clear buffer only
    ret = cfb_print(disp, buf, 0, row*8);          // print at (0,0)
    if (ret < 0) {
        LOG_ERR("Failed to print string: %d", ret);
        return;
    } else {
        // LOG_DBG("String printed to framebuffer with return code: %d", ret);
    }
    cfb_framebuffer_finalize(disp);       // WRITE to display
}

void clear_text()
{
    cfb_framebuffer_clear(disp, true);  // clear buffer and display
    memset((void *)display_buff, 0, sizeof(display_buff));
    cfb_framebuffer_finalize(disp);       // WRITE to display
    return;
}

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

    
    

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return 0;
    }
    // i2c_scan_bus(i2c_dev);


/* Oled Display */
    if (!device_is_ready(disp)) {
        LOG_ERR("Display not ready");
        return 0;
    }

    if (display_set_pixel_format(disp, PIXEL_FORMAT_MONO10) != 0) {
       if (display_set_pixel_format(disp, PIXEL_FORMAT_MONO01) != 0) {
           LOG_ERR("Failed to set Display Format");
           return 0;
       }else{
        LOG_DBG("Set Display format to 01");
       }
   }else{
    LOG_DBG("Set Display format to 10");
   }

    struct display_capabilities caps;
    display_get_capabilities(disp, &caps);
    LOG_DBG("Display caps: %dx%d, format=%d", 
    caps.x_resolution, caps.y_resolution, caps.current_pixel_format);

    ret = cfb_framebuffer_init(disp);
    if (ret < 0) {
        LOG_ERR("Failed to initialize framebuffer: %d", ret);
        return 0;
    }
    ret = cfb_framebuffer_clear(disp, true);
    if (ret < 0) {
        LOG_ERR("Failed to clear framebuffer: %d", ret);
        return 0;
    }
    ret = display_blanking_off(disp);
    if (ret < 0) {
        LOG_ERR("Failed to turn off display blanking: %d", ret);
        return 0;
    }
    display_string("test");
    update_row(0, "Starting");

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

    /* Initialize LoRa device */
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa device not ready");
    } else {
        lora_tx_config.frequency = 868000000;
        lora_tx_config.bandwidth = BW_250_KHZ;
        lora_tx_config.datarate = SF_8;
        lora_tx_config.coding_rate = CR_4_5;
        lora_tx_config.preamble_len = 12;
        lora_tx_config.tx_power = 14;
        lora_tx_config.tx = true;
        lora_tx_config.iq_inverted = false;
        lora_tx_config.public_network = false;

        ret = lora_config(lora_dev, &lora_tx_config);
        if (ret < 0) {
            LOG_ERR("LoRa config failed: %d", ret);
        } else {
            LOG_INF("LoRa initialized: 868 MHz, SF8, 14 dBm");
        }
    }    
    char buf[255] = {0};
    int16_t RSSI;
    int8_t SNR;

    update_row(1,"LoRa ok");

    /* Mount SD card */
    
    // LOG_INF("Initializing SD card");
    // #ifdef CONFIG_FAT_FILESYSTEM_ELM
    // LOG_INF("Mounting SD card...");
    // fatfs_mount();
    // #endif

    /* Neopixel enable */
    gpio_pin_configure_dt(&neopixel_en, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&neopixel_en, 1);

    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    static const struct pwm_dt_spec buzzer =
        PWM_DT_SPEC_GET(DT_NODELABEL(buzzer));
    if (!device_is_ready(buzzer.dev)) {
        LOG_ERR("Buzzer PWM not ready");
        return -ENODEV;
    }
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

    update_row(2, "LED,Sound OK");

    /* Encoder */
    if (!gpio_is_ready_dt(&enc_a) || !gpio_is_ready_dt(&enc_b) ||
        !gpio_is_ready_dt(&enc_sw)) {
        LOG_ERR("Encoder GPIOs not ready");
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



    /* B1 User button */
    if (!gpio_is_ready_dt(&user_btn)) {
        LOG_ERR("User button not ready");
        return 0;
    }
    gpio_pin_configure_dt(&user_btn, GPIO_INPUT);

    ret = gpio_pin_interrupt_configure_dt(&user_btn, GPIO_INT_EDGE_FALLING);

    gpio_init_callback(&btn_cb_data, button_handler, BIT(user_btn.pin));
    ret = gpio_add_callback(user_btn.port, &btn_cb_data);

    update_row(3, "Buttons OK");

    update_row(4, "UDP init");
    udp_client_init();
    update_row(5, "UDP Ok");


    struct gps_position pos;
    if (gps_get_latest(&pos) == 0) {
        LOG_INF("GPS: %s data \n\t\t %d sats\n\t\t %d fix type  ",pos.valid ? "Valid" : "Invalid", pos.satellites, pos.fix_type);
        update_row(6, "GPS: %d sats", pos.satellites);
    } else {
        update_row(6, "GPS: No fix");
    }


    struct led_rgb pixels[NUM_LEDS] = {0};
    bool toggle = false;
    uint8_t tx_buf[] = "Hello from Obelics!";
    

    LOG_INF("Starting main loop...");
    // display_string("Main loop");
    update_row(5, "Init complete");
    k_sleep(K_MSEC(1000));

    clear_text();
    int lora_counter = 0;
    int row=0;
    while (1) {

        // display_string("Running main loop...");
        /* LoRa TX every 5 seconds */
        if (device_is_ready(lora_dev)) {
            int err = lora_send(lora_dev, tx_buf, sizeof(tx_buf));
            if (err == 0) {
                LOG_DBG("LoRa TX #%d: %d bytes", lora_counter++, (int)sizeof(tx_buf));
            } else {
                LOG_ERR("LoRa TX failed: %d", err);
            }
        }

        memset(pixels, 0, sizeof(pixels));
        if (toggle) {
            pixels[0].r = 128;
            pixels[3].r = 128;
            pixels[3].b = 128;
        } else {
            pixels[1].b = 128;
            pixels[2].g = 128;
        }
        toggle = !toggle;

        led_strip_update_rgb(strip, pixels, NUM_LEDS);

        

        int err = lora_recv(lora_dev, buf, sizeof(buf), K_SECONDS(2), &RSSI, &SNR);
        if (err == -EAGAIN) {
            LOG_DBG("No LoRa RX data yet");
            update_row(7, "No LoRa RX data");
        } else if (err < 0) {
            LOG_ERR("LoRa RX failed: %d", err);
            update_row(7, "LoRa RX failed: %d", err);
        } else {
            LOG_INF("LoRa RX: %d bytes: %s", err, buf);
            LOG_INF("RSSI: %d, SNR: %d", RSSI, SNR);
            update_row(7, "RSSI: %d, SNR: %d", RSSI, SNR);
        }
        row = (row + 1) % 8; // cycle through display rows for updates


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
            update_row(0, "%d sats fix=%d", pos.satellites, pos.fix_type);
            update_row(1, "lat%d", pos.latitude);
            update_row(2,"lon%d", pos.longitude);
            update_row(3,"alt%d m", (int)(pos.altitude_mm/1000));
            update_row(4, "%02d:%02d:%02d", pos.hour, pos.minute, pos.second);
        } else {
            update_row(0, "GPS: No fix");
        }
        k_sleep(K_SECONDS(3));
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
