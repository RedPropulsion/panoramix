// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/pwm.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/sys/util.h>
// #include <zephyr/drivers/led_strip.h>
// #include <zephyr/drivers/lora.h>
// #include <zephyr/logging/log.h>


// LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// /* ------------------------------------------------------------------ *
//  * LoRa
//  * ------------------------------------------------------------------ */
// // #define LORA_NODE DT_NODELABEL(lora0)
// // LOG_MODULE_REGISTER(lora_tx, LOG_LEVEL_DBG);

// // BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(LORA_NODE),
// // 	     "No LoRa0 radio specified in DT");




// // #define MAX_DATA_LEN 255

// // void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size,
// // 		     int16_t rssi, int8_t snr, void *user_data)
// // {
// // 	static int cnt;

// // 	ARG_UNUSED(dev);
// // 	ARG_UNUSED(size);
// // 	ARG_UNUSED(user_data);

// // 	LOG_INF("LoRa RX RSSI: %d dBm, SNR: %d dB", rssi, snr);
// // 	// LOG_HEXDUMP_INF(data, size, "LoRa RX payload");

// // 	/* Stop receiving after 10 packets */
// // 	if (++cnt == 10) {
// // 		LOG_INF("Stopping packet receptions");
// // 		lora_recv_async(dev, NULL, NULL);
// // 	}
// // }

// // char data[MAX_DATA_LEN] = {'h', 'e', 'l', 'l', 'o', 'w', 'o', 'r', 'l', 'd', ' ', '0'};

// /* ------------------------------------------------------------------ *
//  * Neopixel
//  * ------------------------------------------------------------------ */
// #define STRIP_NODE  DT_NODELABEL(led_strip)
// #define NUM_LEDS    DT_PROP(STRIP_NODE, chain_length)

// static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
// static const struct gpio_dt_spec neopixel_en =
//     GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_en), gpios);

// /* ------------------------------------------------------------------ *
//  * Buzzer — PWM on PE4 via TIM15_CH1
//  * ------------------------------------------------------------------ */
// static const struct pwm_dt_spec buzzer =
//     PWM_DT_SPEC_GET(DT_NODELABEL(buzzer)); //TODO: rename to buzzer at some point maybe.
// uint32_t period = PWM_HZ(4000);

// /* Simple note definitions — period in nanoseconds */
// #define NOTE_C6   477274U   /* 2093 Hz */
// #define NOTE_D6   425637U   /* 2349 Hz */
// #define NOTE_E6   379245U   /* 2637 Hz */
// #define NOTE_F6   357858U   /* 2794 Hz */
// #define NOTE_G6   318878U   /* 3136 Hz */
// #define NOTE_A6   284091U   /* 3520 Hz */
// #define NOTE_B6   253099U   /* 3951 Hz */
// #define NOTE_C7   238906U   /* 4186 Hz ← closest to resonance */
// #define NOTE_REST 0U        /* silence */

// struct note {
//     uint32_t period_ns;
//     uint32_t duration_ms;
// };

// /* Simple tune — first 8 notes of Ode to Joy */
// /* ON/OFF durations in ms — rhythm encodes the melody */
// // static const struct note tune[] = {
// //     {200}, {200}, {400}, {200}, {200},   /* shave and a haircut */
// //     {0, 200},                             /* pause */
// //     {200}, {200},                         /* two bits */
// // };

// /* Semaphore — button ISR signals buzzer thread */
// K_SEM_DEFINE(buzzer_sem, 0, 1);

// /* ------------------------------------------------------------------ *
//  * Buzzer thread
//  * ------------------------------------------------------------------ */
// #define BUZZER_STACK_SIZE 512
// #define BUZZER_PRIORITY   5

// void buzzer_thread_fn(void *a, void *b, void *c)
// {
//     ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

//     while (1) {
//         k_sem_take(&buzzer_sem, K_FOREVER);
//         printk("Beeping...\n");


        
//         pwm_set_dt(&buzzer,  buzzer.period, buzzer.period /2U);  /* ON */
//         k_sleep(K_MSEC(1000));
//         pwm_set_dt(&buzzer,  buzzer.period, 0);                  /* OFF */


//         printk("Beeps done.\n");
//     }
// }

// #define BUZZER_FREQ_NS  250000U   /* 4000 Hz = 250µs period */

// K_THREAD_DEFINE(buzzer_thread, BUZZER_STACK_SIZE,
//                 buzzer_thread_fn, NULL, NULL, NULL,
//                 BUZZER_PRIORITY, 0, 0);

// /* ------------------------------------------------------------------ *
//  * Button
//  * ------------------------------------------------------------------ */
// static const struct gpio_dt_spec user_btn =
//     GPIO_DT_SPEC_GET(DT_NODELABEL(user_button), gpios);

// static struct gpio_callback btn_cb_data;

// void button_handler(const struct device *dev, struct gpio_callback *cb,
//                     uint32_t pins)
// {
//     /* Debounce */
//     static uint32_t last_time = 0;
//     uint32_t now = k_uptime_get_32();
//     if (now - last_time < 200) return;
//     last_time = now;

//     printk("Button pressed!\n");
//     k_sem_give(&buzzer_sem);  /* wake buzzer thread */
// }

// /* ------------------------------------------------------------------ *
//  * LED blink timers (unchanged)
//  * ------------------------------------------------------------------ */
// struct led_data {
//     struct gpio_dt_spec gpio;
//     struct k_timer timer;
//     int index;
// };

// static struct led_data leds[] = {
//     { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios), .index = 0 },
//     { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios), .index = 1 },
//     { .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios), .index = 2 }
// };

// volatile int selected_led = 0;
// volatile uint32_t intervals[] = {500, 500, 500};

// void led_timer_handler(struct k_timer *timer_id) {
//     struct led_data *led = CONTAINER_OF(timer_id, struct led_data, timer);
//     gpio_pin_toggle_dt(&led->gpio);
//     k_timer_start(&led->timer, K_MSEC(intervals[led->index]), K_NO_WAIT);
// }

// /* ------------------------------------------------------------------ *
//  * Encoder (unchanged)
//  * ------------------------------------------------------------------ */
// static const struct gpio_dt_spec enc_a  = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_a), gpios);
// static const struct gpio_dt_spec enc_b  = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_b), gpios);
// static const struct gpio_dt_spec enc_sw = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_s), gpios);

// static struct gpio_callback enc_a_cb_data;
// static struct gpio_callback enc_sw_cb_data;

// void enc_sw_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
//     static uint32_t last_time = 0;
//     uint32_t now = k_uptime_get_32();
//     if (now - last_time < 50) return;
//     last_time = now;
//     selected_led = (selected_led + 1) % ARRAY_SIZE(leds);
//     printk("\n---> Now adjusting LED %d <---\n", selected_led);
// }

// void encoder_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
//     int phase_sw = gpio_pin_get_dt(&enc_sw);
//     int step = 10;
//     if (!phase_sw) {
//         static uint32_t last_time = 0;
//         uint32_t now = k_uptime_get_32();
//         if (now - last_time < 5) return;
//         last_time = now;
//         int phase_b = gpio_pin_get_dt(&enc_b);
//         if (phase_b) {
//             intervals[selected_led] += step;
//         } else {
//             if (intervals[selected_led] > step)
//                 intervals[selected_led] -= step;
//         }
//         printk("LED sel=%d intervals: %d \t %d \t %d ms\n",
//                selected_led, intervals[0], intervals[1], intervals[2]);
//     } else {
//         enc_sw_handler(dev, cb, pins);
//     }
// }

// /* ------------------------------------------------------------------ *
//  * Main
//  * ------------------------------------------------------------------ */
// int main(void)
// {   
//     /* LoRa */
//     // const struct device *const lora_dev = DEVICE_DT_GET(LORA_NODE);
//     // struct lora_modem_config config;
//     int ret;
//     // int len;

//     // Receive buffer and metadata
//     // uint8_t data[MAX_DATA_LEN] = {0};
// 	// int16_t rssi;
// 	// int8_t snr;
    
//     // if (!device_is_ready(lora_dev)) {
// 	// 	printk("LoRa Device not ready");
// 	// 	return 0;
// 	// }

// 	// config.frequency = 868000000;
// 	// config.bandwidth = BW_125_KHZ;
// 	// config.datarate = SF_7;
// 	// config.preamble_len = 12;
// 	// config.coding_rate = CR_4_5;
// 	// config.iq_inverted = false;
// 	// config.public_network = false;
// 	// config.tx_power = 4;
// 	// config.tx = false;


//     // ret = lora_config(lora_dev, &config);
// 	// if (ret < 0) {
//     //     printk("LoRa config failed\n");
// 	// 	return 0;
// 	// }    

//     /* Neopixel enable */
//     gpio_pin_configure_dt(&neopixel_en, GPIO_OUTPUT_ACTIVE);
//     gpio_pin_set_dt(&neopixel_en, 1);

//     if (!device_is_ready(strip)) {
//         printk("LED strip not ready\n");
//         return -ENODEV;
//     }

//     /* Buzzer */
//     if (!device_is_ready(buzzer.dev)) {
//         printk("Buzzer PWM not ready\n");
//         return -ENODEV;
//     }

//     pwm_set_dt(&buzzer,  buzzer.period, 0);


    
    
//     /* LEDs */
//     for (int i = 0; i < ARRAY_SIZE(leds); i++) {
//         if (!gpio_is_ready_dt(&leds[i].gpio)) {
//             printk("Error: LED %d not ready\n", i);
//             return 0;
//         }
//         gpio_pin_configure_dt(&leds[i].gpio, GPIO_OUTPUT_ACTIVE);
//         k_timer_init(&leds[i].timer, led_timer_handler, NULL);
//         k_timer_start(&leds[i].timer, K_MSEC(intervals[i]), K_NO_WAIT);
//     }

//     /* Encoder */
//     if (!gpio_is_ready_dt(&enc_a) || !gpio_is_ready_dt(&enc_b) ||
//         !gpio_is_ready_dt(&enc_sw)) {
//         printk("Error: Encoder GPIOs not ready\n");
//         return 0;
//     }
//     gpio_pin_configure_dt(&enc_a, GPIO_INPUT);
//     gpio_pin_configure_dt(&enc_b, GPIO_INPUT);
//     gpio_pin_configure_dt(&enc_sw, GPIO_INPUT);

//     gpio_pin_interrupt_configure_dt(&enc_a, GPIO_INT_EDGE_RISING);
//     gpio_init_callback(&enc_a_cb_data, encoder_handler, BIT(enc_a.pin));
//     gpio_add_callback(enc_a.port, &enc_a_cb_data);

//     gpio_pin_interrupt_configure_dt(&enc_sw, GPIO_INT_EDGE_RISING);
//     gpio_init_callback(&enc_sw_cb_data, encoder_handler, BIT(enc_sw.pin));
//     gpio_add_callback(enc_sw.port, &enc_sw_cb_data);

//     /* B1 User button */
//     if (!gpio_is_ready_dt(&user_btn)) {
//         printk("Error: Button not ready\n");
//         return 0;
//     }
//     gpio_pin_configure_dt(&user_btn, GPIO_INPUT);

//     /* Verify you can actually read the pin */
//     printk("Button init done. Current pin state: %d\n", gpio_pin_get_dt(&user_btn));

//     ret = gpio_pin_interrupt_configure_dt(&user_btn, GPIO_INT_EDGE_FALLING);
//     printk("Interrupt configure returned: %d\n", ret);

//     gpio_init_callback(&btn_cb_data, button_handler, BIT(user_btn.pin));
//     ret = gpio_add_callback(user_btn.port, &btn_cb_data);
//     printk("Add callback returned: %d\n", ret);

//     printk("Starting main loop...\n");

//     struct led_rgb pixels[NUM_LEDS] = {0};
//     bool toggle = false;


//     // LOG_INF("Synchronous reception");

//     while (1) {
//         /* Block until data arrives */
// 		// len = lora_recv(lora_dev, data, MAX_DATA_LEN, K_MSEC(1000),
// 		// 		&rssi, &snr);
// 		// if (len < 0) {
// 		// 	LOG_ERR("LoRa receive failed");
// 		// } else{
//         //     LOG_INF("LoRa RX RSSI: %d dBm, SNR: %d dB", rssi, snr);
//         //     LOG_HEXDUMP_INF(data, len, "LoRa RX payload");
//         // }
//         // ret = lora_send(lora_dev, data, MAX_DATA_LEN);
//         // if (ret < 0) {
//         //     printk("LoRa send failed\n");
// 		// 	return 0;
// 		// }

//         // printk("Data sent %c!\n", data[MAX_DATA_LEN - 1]);

//         // /* Increment final character to differentiate packets */
// 		// if (data[MAX_DATA_LEN - 1] == '9') {
// 		// 	data[MAX_DATA_LEN - 1] = '0';
// 		// } else {
// 		// 	data[MAX_DATA_LEN - 1] += 1;
// 		// }

//         memset(pixels, 0, sizeof(pixels));
//         if (toggle) {
//             pixels[0].r = 128;
//         } else {
//             pixels[1].b = 128;
//         }
//         toggle = !toggle;

//         led_strip_update_rgb(strip, pixels, NUM_LEDS);
//         k_sleep(K_SECONDS(1));
//     }
//     return 0;
// }


////////////////////////////////////////////////////////////////////////////////
/*
 * SX127x LoRa driver for Zephyr
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lora_tx, LOG_LEVEL_DBG);

/* 1. Binding Hardware a tempo di compilazione */
static const struct device *lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora_sx1261));
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
    printk("Starting LoRa TX example...\n");
    struct lora_modem_config config;
    int err;
    uint8_t tx_buf[] = "Test Zephyr 4dBm"; /* Payload da trasmettere */

    /* 2. Verifica dispositivi e GPIO */
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("Modulo LoRa non pronto!");
        return -1;
    }

    // if (!gpio_is_ready_dt(&led)) {
    //     LOG_ERR("Controller GPIO per il LED non pronto!");
    //     return -1;
    // }

    // gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    /* 3. Configurazione Modem RF per Trasmissione */
    config.frequency = 868000000;
    config.bandwidth = BW_125_KHZ;
    config.datarate = SF_7;
    config.coding_rate = CR_4_5;
    config.preamble_len = 12;
    
    /* MODIFICA: Imposta la potenza di trasmissione a 4 dBm */
    config.tx_power = 4; 
    
    /* MODIFICA: Indica al driver di preparare la catena RF per la trasmissione */
    config.tx = true; 
    
    config.iq_inverted = false;
    config.public_network = false;

    err = lora_config(lora_dev, &config);
    if (err < 0) {
        LOG_ERR("Errore configurazione LoRa: %d", err);
        return -1;
    }

    // LOG_INF("Inizio trasmissione su 868 MHz a 4 dBm...");

    /* 4. Ciclo di Trasmissione Continuo */
    while (1) {
        /* lora_send() trasferisce i dati al buffer FIFO via SPI (o bus interno su STM32WL)
         * e innesca la modalità TX impostando il registro OPMODE.
         */
        err = lora_send(lora_dev, tx_buf, sizeof(tx_buf));
        
        if (err == 0) {
            LOG_INF("Inviati %d bytes con successo.", sizeof(tx_buf));
        } else {
            LOG_ERR("Errore in trasmissione: %d", err);
        }

        /* Sleep per sospendere il thread e rispettare il Duty Cycle.
         * Previene l'errore -11 (-EAGAIN) descritto in precedenza. 
         */
        k_sleep(K_SECONDS(1));
    }
}
