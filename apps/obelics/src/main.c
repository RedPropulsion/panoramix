#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
// #include <ff.h>
#include <zephyr/storage/disk_access.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

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
// #define LORA_NODE DT_NODELABEL(lora0)
// LOG_MODULE_REGISTER(lora_tx, LOG_LEVEL_DBG);

// BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(LORA_NODE),
// 	     "No LoRa0 radio specified in DT");




// #define MAX_DATA_LEN 255

// void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size,
// 		     int16_t rssi, int8_t snr, void *user_data)
// {
// 	static int cnt;

// 	ARG_UNUSED(dev);
// 	ARG_UNUSED(size);
// 	ARG_UNUSED(user_data);

// 	LOG_INF("LoRa RX RSSI: %d dBm, SNR: %d dB", rssi, snr);
// 	// LOG_HEXDUMP_INF(data, size, "LoRa RX payload");

// 	/* Stop receiving after 10 packets */
// 	if (++cnt == 10) {
// 		LOG_INF("Stopping packet receptions");
// 		lora_recv_async(dev, NULL, NULL);
// 	}
// }

// char data[MAX_DATA_LEN] = {'h', 'e', 'l', 'l', 'o', 'w', 'o', 'r', 'l', 'd', ' ', '0'};

/* ------------------------------------------------------------------ *
 * Neopixel
 * ------------------------------------------------------------------ */
#define STRIP_NODE  DT_NODELABEL(led_strip)
#define NUM_LEDS    DT_PROP(STRIP_NODE, chain_length)

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec neopixel_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_en), gpios);

/* ------------------------------------------------------------------ *
 * Buzzer — PWM on PE4 via TIM15_CH1
 * ------------------------------------------------------------------ */
static const struct pwm_dt_spec buzzer =
    PWM_DT_SPEC_GET(DT_NODELABEL(buzzer)); //TODO: rename to buzzer at some point maybe.
uint32_t period = PWM_HZ(4000);

/* Simple note definitions — period in nanoseconds */
#define NOTE_E5  659
#define NOTE_C5  523
#define NOTE_G5  784
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_AS4 466
#define NOTE_F5  698
#define NOTE_E4  330
#define NOTE_A5  880
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_REST 0

#define BPM        200
#define BEAT_MS    (60000 / BPM)
#define Q          BEAT_MS          /* quarter note */
#define E          (BEAT_MS / 2)    /* eighth note */
#define H          (BEAT_MS * 2)    /* half note */

struct note {
    uint32_t period_ns;
    uint32_t duration_ms;
};

typedef struct { uint32_t freq_hz; uint32_t dur_ms; } Note;

static const Note mario_melody[] = {
    {NOTE_E5, E}, {NOTE_E5, E}, {NOTE_REST, E}, {NOTE_E5, E},
    {NOTE_REST, E}, {NOTE_C5, E}, {NOTE_E5, Q},
    {NOTE_G5, Q}, {NOTE_REST, Q}, {NOTE_G4, Q},
    {NOTE_REST, Q},

    {NOTE_C5, Q}, {NOTE_REST, E}, {NOTE_G4, E},
    {NOTE_REST, Q}, {NOTE_E4, Q},
    {NOTE_A4, E}, {NOTE_REST, E}, {NOTE_B4, E},
    {NOTE_REST, E}, {NOTE_AS4, E}, {NOTE_A4, Q},

    {NOTE_G4, E}, {NOTE_E5, E}, {NOTE_G5, E},
    {NOTE_A5, Q}, {NOTE_F5, E}, {NOTE_G5, E},
    {NOTE_REST, E}, {NOTE_E5, Q},
    {NOTE_C5, E}, {NOTE_D5, E}, {NOTE_B4, Q},
};

/* Semaphore — button ISR signals buzzer thread */
K_SEM_DEFINE(buzzer_sem, 0, 1);

/* ------------------------------------------------------------------ *
 * Buzzer thread
 * ------------------------------------------------------------------ */
#define BUZZER_STACK_SIZE 512
#define BUZZER_PRIORITY   5


static void play_note(uint32_t freq_hz, uint32_t dur_ms)
{
    if (freq_hz == NOTE_REST || freq_hz == 0) {
        pwm_set_dt(&buzzer, buzzer.period, 0);
    } else {
        uint32_t period_ns = NSEC_PER_SEC / freq_hz;
        pwm_set_dt(&buzzer, period_ns, period_ns / 2U);
    }
    k_sleep(K_MSEC(dur_ms));
    /* Brief gap between notes for articulation */
    pwm_set_dt(&buzzer, buzzer.period, 0);
    k_sleep(K_MSEC(10));
}

void buzzer_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    while (1) {
        k_sem_take(&buzzer_sem, K_FOREVER);
        printk("Playing Mario tune...\n");

        for (size_t i = 0; i < ARRAY_SIZE(mario_melody); i++) {
            play_note(mario_melody[i].freq_hz, mario_melody[i].dur_ms);
        }

        pwm_set_dt(&buzzer, buzzer.period, 0); /* ensure OFF */
        printk("Tune done.\n");
    }
}



#define BUZZER_FREQ_NS  250000U   /* 4000 Hz = 250µs period */

K_THREAD_DEFINE(buzzer_thread, BUZZER_STACK_SIZE,
                buzzer_thread_fn, NULL, NULL, NULL,
                BUZZER_PRIORITY, 0, 0);

/* ------------------------------------------------------------------ *
 * Button
 * ------------------------------------------------------------------ */
static const struct gpio_dt_spec user_btn =
    GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);

static struct gpio_callback btn_cb_data;

void button_handler(const struct device *dev, struct gpio_callback *cb,
                    uint32_t pins)
{
    /* Debounce */
    static uint32_t last_time = 0;
    uint32_t now = k_uptime_get_32();
    if (now - last_time < 200) return;
    last_time = now;

    printk("Button pressed!\n");
    k_sem_give(&buzzer_sem);  /* wake buzzer thread */
}

/* ------------------------------------------------------------------ *
 * LED blink timers (unchanged)
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
    printk("\n---> Now adjusting LED %d <---\n", selected_led);
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
        printk("LED sel=%d intervals: %d \t %d \t %d ms\n",
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
     printk("=== main() entered ===\n");  // add this immediately
    /* LoRa */
    // const struct device *const lora_dev = DEVICE_DT_GET(LORA_NODE);
    // struct lora_modem_config config;
    int ret;
    // int len;

    // Receive buffer and metadata
    // uint8_t data[MAX_DATA_LEN] = {0};
	// int16_t rssi;
	// int8_t snr;
    
    // if (!device_is_ready(lora_dev)) {
	// 	printk("LoRa Device not ready");
	// 	return 0;
	// }

	// config.frequency = 868000000;
	// config.bandwidth = BW_125_KHZ;
	// config.datarate = SF_7;
	// config.preamble_len = 12;
	// config.coding_rate = CR_4_5;
	// config.iq_inverted = false;
	// config.public_network = false;
	// config.tx_power = 4;
	// config.tx = false;


    // ret = lora_config(lora_dev, &config);
	// if (ret < 0) {
    //     printk("LoRa config failed\n");
	// 	return 0;
	// }    

    /* Mount SD card */
    LOG_INF("Initializing SD card... \n if CONFIG_FAT_FILESYSTEM_ELM is enabled, you should see SD card info below");
    #ifdef CONFIG_FAT_FILESYSTEM_ELM
    LOG_INF("Mounting SD card...");
    fatfs_mount();
    #endif
    

    /* Neopixel enable */
    gpio_pin_configure_dt(&neopixel_en, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&neopixel_en, 1);

    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    /* Buzzer */
    if (!device_is_ready(buzzer.dev)) {
        LOG_ERR("Buzzer PWM not ready");
        return -ENODEV;
    }

    pwm_set_dt(&buzzer,  buzzer.period, 0);


    
    
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
    LOG_DBG("Interrupt configure returned: %d\n", ret);

    gpio_init_callback(&btn_cb_data, button_handler, BIT(user_btn.pin));
    ret = gpio_add_callback(user_btn.port, &btn_cb_data);
    // LOG_DBG("Add callback returned: %d\n", ret);

    LOG_INF("Starting main loop...\n");

    struct led_rgb pixels[NUM_LEDS] = {0};
    bool toggle = false;


    // LOG_INF("Synchronous reception");

    while (1) {


        memset(pixels, 0, sizeof(pixels));
        if (toggle) {
            pixels[0].r = 128;
        } else {
            pixels[1].b = 128;
        }
        toggle = !toggle;

        led_strip_update_rgb(strip, pixels, NUM_LEDS);
        k_sleep(K_SECONDS(1));
    }
    return 0;
}


////////////////////////////////////////////////////////////////////////////////
/*
 * SX127x LoRa driver for Zephyr
 */

// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/lora.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/logging/log.h>

// LOG_MODULE_REGISTER(lora_tx, LOG_LEVEL_DBG);

// /* 1. Binding Hardware a tempo di compilazione */
// static const struct device *lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora_sx1276));
// // static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// int main(void)
// {
//     printk("Starting LoRa TX example...\n");
//     struct lora_modem_config config;
//     int err;
//     uint8_t tx_buf[] = "Test Zephyr 4dBm"; /* Payload da trasmettere */

//     /* 2. Verifica dispositivi e GPIO */
//     if (!device_is_ready(lora_dev)) {
//         LOG_ERR("Modulo LoRa non pronto!");
//         return -1;
//     }

//     // if (!gpio_is_ready_dt(&led)) {
//     //     LOG_ERR("Controller GPIO per il LED non pronto!");
//     //     return -1;
//     // }

//     // gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

//     /* 3. Configurazione Modem RF per Trasmissione */
//     config.frequency = 868000000;
//     config.bandwidth = BW_125_KHZ;
//     config.datarate = SF_7;
//     config.coding_rate = CR_4_5;
//     config.preamble_len = 12;
    
//     /* MODIFICA: Imposta la potenza di trasmissione a 4 dBm */
//     config.tx_power = 4; 
    
//     /* MODIFICA: Indica al driver di preparare la catena RF per la trasmissione */
//     config.tx = true; 
    
//     config.iq_inverted = false;
//     config.public_network = false;

//     err = lora_config(lora_dev, &config);
//     if (err < 0) {
//         LOG_ERR("Errore configurazione LoRa: %d", err);
//         return -1;
//     }

//     LOG_INF("Inizio trasmissione su 868 MHz a 4 dBm...");

//     /* 4. Ciclo di Trasmissione Continuo */
//     while (1) {
//         /* lora_send() trasferisce i dati al buffer FIFO via SPI (o bus interno su STM32WL)
//          * e innesca la modalità TX impostando il registro OPMODE.
//          */
//         err = lora_send(lora_dev, tx_buf, sizeof(tx_buf));
        
//         if (err == 0) {
//             LOG_INF("Inviati %d bytes con successo.", sizeof(tx_buf));
//         } else {
//             LOG_ERR("Errore in trasmissione: %d", err);
//         }

//         /* Sleep per sospendere il thread e rispettare il Duty Cycle.
//          * Previene l'errore -11 (-EAGAIN) descritto in precedenza. 
//          */
//         k_sleep(K_SECONDS(1));
//     }
// }
