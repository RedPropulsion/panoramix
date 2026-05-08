#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>

#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/drivers/led_strip.h>
#include "sound.h"
#include "udp_client.h"

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
 * Main
 * ------------------------------------------------------------------ */
int main(void)
{   
     LOG_INF("Starting main()");
    int ret;

    
    udp_client_init();

    /* Initialize LoRa device */
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa device not ready");
    } else {
        lora_tx_config.frequency = 868000000;
        lora_tx_config.bandwidth = BW_125_KHZ;
        lora_tx_config.datarate = SF_7;
        lora_tx_config.coding_rate = CR_4_5;
        lora_tx_config.preamble_len = 12;
        lora_tx_config.tx_power = 4;
        lora_tx_config.tx = true;
        lora_tx_config.iq_inverted = false;
        lora_tx_config.public_network = false;

        ret = lora_config(lora_dev, &lora_tx_config);
        if (ret < 0) {
            LOG_ERR("LoRa config failed: %d", ret);
        } else {
            LOG_INF("LoRa initialized: 868 MHz, SF7, 4 dBm");
        }
    }

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

    /* Verify you can actually read the pin */
    // LOG_DBG("Button init done. Current pin state: %d\n", gpio_pin_get_dt(&user_btn));

    ret = gpio_pin_interrupt_configure_dt(&user_btn, GPIO_INT_EDGE_FALLING);

    gpio_init_callback(&btn_cb_data, button_handler, BIT(user_btn.pin));
    ret = gpio_add_callback(user_btn.port, &btn_cb_data);



    struct led_rgb pixels[NUM_LEDS] = {0};
    bool toggle = false;
    uint8_t tx_buf[] = "Hello LoRa!";
    int lora_counter = 0;


    LOG_INF("Starting main loop...\n");
    while (1) {
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
        } else {
            pixels[1].b = 128;
        }
        toggle = !toggle;

        led_strip_update_rgb(strip, pixels, NUM_LEDS);
        k_sleep(K_SECONDS(5));
    }
    return 0;
}
