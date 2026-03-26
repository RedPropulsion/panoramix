/*
 * main.c — SchedaRF Board Test
 *
 * Testa le seguenti periferiche definite nel DTS:
 *   - LED GPIO (led_ring + led_rf_1..7)
 *   - Button (user_button su PC13)
 *   - USART3 (console/debug VCP)
 *   - USART1 (comunicazione Asterics MCU)
 *   - USART2 (comunicazione ESP32S3)
 *   - SPI2 + LoRa SX1262 (tramite driver Zephyr lora)
 *
 * Console/Shell: USART3 @ 115200 baud
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(rf_board_test, LOG_LEVEL_DBG);

/* -----------------------------------------------------------------------
 * DTS aliases / nodelabels
 * --------------------------------------------------------------------- */

/* LED ring */
#define LED_RING_NODE   DT_NODELABEL(led_ring)
/* LED RF individuali */
#define LED_RF1_NODE    DT_NODELABEL(led_rf_1)
#define LED_RF2_NODE    DT_NODELABEL(led_rf_2)
#define LED_RF3_NODE    DT_NODELABEL(led_rf_3)
#define LED_RF4_NODE    DT_NODELABEL(led_rf_4)
#define LED_RF5_NODE    DT_NODELABEL(led_rf_5)
#define LED_RF6_NODE    DT_NODELABEL(led_rf_6)
#define LED_RF7_NODE    DT_NODELABEL(led_rf_7)

/* Button */
#define BTN_NODE        DT_NODELABEL(user_button)

/* UART */
#define UART1_NODE      DT_NODELABEL(usart1)
#define UART2_NODE      DT_NODELABEL(usart2)
#define UART3_NODE      DT_NODELABEL(usart3)   /* console */

/* LoRa */
#define LORA_NODE       DT_NODELABEL(lora_sx1262_asterics)

/* -----------------------------------------------------------------------
 * GPIO specs
 * --------------------------------------------------------------------- */

static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(LED_RING_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_RF1_NODE,  gpios),
    GPIO_DT_SPEC_GET(LED_RF2_NODE,  gpios),
    GPIO_DT_SPEC_GET(LED_RF3_NODE,  gpios),
    GPIO_DT_SPEC_GET(LED_RF4_NODE,  gpios),
    GPIO_DT_SPEC_GET(LED_RF5_NODE,  gpios),
    GPIO_DT_SPEC_GET(LED_RF6_NODE,  gpios),
    GPIO_DT_SPEC_GET(LED_RF7_NODE,  gpios),
};
#define NUM_LEDS ARRAY_SIZE(leds)

static const char *led_names[] = {
    "LED_RING",
    "LED_RF1", "LED_RF2", "LED_RF3", "LED_RF4",
    "LED_RF5", "LED_RF6", "LED_RF7",
};

static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(BTN_NODE, gpios);

/* -----------------------------------------------------------------------
 * Button callback
 * --------------------------------------------------------------------- */

static struct gpio_callback btn_cb_data;
static volatile bool btn_pressed = false;

static void button_pressed_cb(const struct device *dev,
                               struct gpio_callback *cb,
                               uint32_t pins)
{
    btn_pressed = true;
    LOG_INF(">>> Button premuto! <<<");
}

/* -----------------------------------------------------------------------
 * Helpers
 * --------------------------------------------------------------------- */

/** Invia una stringa su una UART (polling). */
static void uart_print(const struct device *dev, const char *msg)
{
    if (!device_is_ready(dev)) {
        return;
    }
    while (*msg) {
        uart_poll_out(dev, *msg++);
    }
}

/* -----------------------------------------------------------------------
 * Test: LED
 * Accende ogni LED per 200 ms in sequenza, poi li spegne tutti.
 * --------------------------------------------------------------------- */

static void test_leds(void)
{
    LOG_INF("=== TEST LED ===");

    /* Inizializzazione */
    for (int i = 0; i < NUM_LEDS; i++) {
        if (!gpio_is_ready_dt(&leds[i])) {
            LOG_ERR("  LED %s: GPIO non pronto", led_names[i]);
            continue;
        }
        if (gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE) < 0) {
            LOG_ERR("  LED %s: configurazione fallita", led_names[i]);
        }
    }

    /* Sequenza lampeggio */
    for (int i = 0; i < NUM_LEDS; i++) {
        LOG_INF("  Accendo %s", led_names[i]);
        gpio_pin_set_dt(&leds[i], 1);
        k_msleep(200);
        gpio_pin_set_dt(&leds[i], 0);
    }

    /* Knight-rider veloce */
    LOG_INF("  Knight-rider...");
    for (int rep = 0; rep < 3; rep++) {
        for (int i = 0; i < NUM_LEDS; i++) {
            gpio_pin_set_dt(&leds[i], 1);
            k_msleep(60);
            gpio_pin_set_dt(&leds[i], 0);
        }
        for (int i = NUM_LEDS - 1; i >= 0; i--) {
            gpio_pin_set_dt(&leds[i], 1);
            k_msleep(60);
            gpio_pin_set_dt(&leds[i], 0);
        }
    }

    /* Tutti ON -> tutti OFF */
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_pin_set_dt(&leds[i], 1);
    }
    k_msleep(500);
    for (int i = 0; i < NUM_LEDS; i++) {
        gpio_pin_set_dt(&leds[i], 0);
    }

    LOG_INF("  Test LED completato.");
}

/* -----------------------------------------------------------------------
 * Test: Button
 * Attende la pressione del tasto per max 10 s.
 * --------------------------------------------------------------------- */

static void test_button(void)
{
    LOG_INF("=== TEST BUTTON ===");

    if (!gpio_is_ready_dt(&btn)) {
        LOG_ERR("  Button GPIO non pronto");
        return;
    }

    if (gpio_pin_configure_dt(&btn, GPIO_INPUT) < 0) {
        LOG_ERR("  Configurazione button fallita");
        return;
    }

    if (gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_EDGE_TO_ACTIVE) < 0) {
        LOG_ERR("  Configurazione interrupt button fallita");
        return;
    }

    gpio_init_callback(&btn_cb_data, button_pressed_cb, BIT(btn.pin));
    gpio_add_callback(btn.port, &btn_cb_data);

    LOG_INF("  Premi il bottone entro 10 secondi...");
    btn_pressed = false;

    int timeout_ms = 10000;
    while (!btn_pressed && timeout_ms > 0) {
        k_msleep(100);
        timeout_ms -= 100;
    }

    if (btn_pressed) {
        LOG_INF("  Button OK — pressione rilevata.");
        /* Feedback visivo sul LED ring */
        for (int i = 0; i < 3; i++) {
            gpio_pin_set_dt(&leds[0], 1);
            k_msleep(100);
            gpio_pin_set_dt(&leds[0], 0);
            k_msleep(100);
        }
    } else {
        LOG_WRN("  Timeout: nessuna pressione rilevata.");
    }

    gpio_remove_callback(btn.port, &btn_cb_data);
}

/* -----------------------------------------------------------------------
 * Test: UART
 * USART1 (Asterics) e USART2 (ESP32) inviano un messaggio di test.
 * USART3 è già usato come console.
 * --------------------------------------------------------------------- */

static void test_uart(void)
{
    LOG_INF("=== TEST UART ===");

    const struct device *uart1 = DEVICE_DT_GET(UART1_NODE);
    const struct device *uart2 = DEVICE_DT_GET(UART2_NODE);
    const struct device *uart3 = DEVICE_DT_GET(UART3_NODE);

    /* USART1 */
    if (device_is_ready(uart1)) {
        const char *msg1 = "\r\n[USART1] SchedaRF -> Asterics MCU: TEST OK\r\n";
        uart_print(uart1, msg1);
        LOG_INF("  USART1 (Asterics): messaggio inviato");
    } else {
        LOG_ERR("  USART1 non pronto");
    }

    /* USART2 */
    if (device_is_ready(uart2)) {
        const char *msg2 = "\r\n[USART2] SchedaRF -> ESP32S3: TEST OK\r\n";
        uart_print(uart2, msg2);
        LOG_INF("  USART2 (ESP32S3): messaggio inviato");
    } else {
        LOG_ERR("  USART2 non pronto");
    }

    /* USART3 (console) */
    if (device_is_ready(uart3)) {
        uart_print(uart3, "\r\n[USART3] Console VCP: TEST OK\r\n");
        LOG_INF("  USART3 (console VCP): OK");
    } else {
        LOG_ERR("  USART3 non pronto");
    }
}

/* -----------------------------------------------------------------------
 * Test: LoRa SX1262
 * Configura il radio, invia un pacchetto di prova e tenta una ricezione
 * di 5 secondi.
 * --------------------------------------------------------------------- */

static void test_lora(void)
{
    LOG_INF("=== TEST LORA SX1262 ===");

    const struct device *lora_dev = DEVICE_DT_GET(LORA_NODE);

    if (!device_is_ready(lora_dev)) {
        LOG_ERR("  Dispositivo LoRa non pronto");
        return;
    }

    /* Configurazione radio */
    struct lora_modem_config cfg = {
        .frequency      = 868100000,   /* 868.1 MHz — banda EU868 */
        .bandwidth      = BW_125_KHZ,
        .datarate       = SF_7,
        .preamble_len   = 8,
        .coding_rate    = CR_4_5,
        .tx_power       = 14,          /* dBm */
        .tx             = true,
    };

    if (lora_config(lora_dev, &cfg) < 0) {
        LOG_ERR("  Configurazione LoRa fallita");
        return;
    }
    LOG_INF("  LoRa configurato: 868.1 MHz, SF7, BW125, CR4/5, 14 dBm");

    /* TX */
    const char tx_payload[] = "SchedaRF_TEST";
    int ret = lora_send(lora_dev,
                        (uint8_t *)tx_payload,
                        strlen(tx_payload));
    if (ret < 0) {
        LOG_ERR("  lora_send fallito: %d", ret);
    } else {
        LOG_INF("  TX OK — payload: \"%s\" (%d byte)", tx_payload,
                strlen(tx_payload));
        /* Feedback LED */
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < NUM_LEDS; j++) {
                gpio_pin_set_dt(&leds[j], 1);
            }
            k_msleep(150);
            for (int j = 0; j < NUM_LEDS; j++) {
                gpio_pin_set_dt(&leds[j], 0);
            }
            k_msleep(150);
        }
    }

    /* RX (modalità non-bloccante con timeout 5 s) */
    cfg.tx = false;
    if (lora_config(lora_dev, &cfg) < 0) {
        LOG_ERR("  Configurazione RX LoRa fallita");
        return;
    }

    uint8_t rx_buf[64] = {0};
    int16_t rssi;
    int8_t  snr;

    LOG_INF("  In ascolto per 5 s...");
    ret = lora_recv(lora_dev, rx_buf, sizeof(rx_buf) - 1,
                    K_SECONDS(5), &rssi, &snr);
    if (ret > 0) {
        rx_buf[ret] = '\0';
        LOG_INF("  RX OK — %d byte: \"%s\" | RSSI=%d dBm | SNR=%d dB",
                ret, rx_buf, rssi, snr);
    } else if (ret == 0 || ret == -EAGAIN) {
        LOG_WRN("  Timeout RX: nessun pacchetto ricevuto (normale in test standalone)");
    } else {
        LOG_ERR("  lora_recv errore: %d", ret);
    }
}

/* -----------------------------------------------------------------------
 * main
 * --------------------------------------------------------------------- */

int main(void)
{
    LOG_INF("╔══════════════════════════════════════╗");
    LOG_INF("║   SchedaRF — Board Bring-Up Test     ║");
    LOG_INF("║   STM32L412RB @ 80 MHz               ║");
    LOG_INF("╚══════════════════════════════════════╝");

    k_msleep(200); /* attendi stabilizzazione clock */

    /* 1. LED */
    test_leds();
    k_msleep(500);

    /* 2. Button */
    test_button();
    k_msleep(200);

    /* 3. UART */
    test_uart();
    k_msleep(200);

    /* 4. LoRa */
    test_lora();

    /* ---- Fine test ---- */
    LOG_INF("=== TEST COMPLETATO ===");
    LOG_INF("Tutti i test eseguiti. Loop LED indicatore di vita...");

    /* LED ring batte ogni secondo per indicare che il firmware è vivo */
    while (1) {
        gpio_pin_set_dt(&leds[0], 1);
        k_msleep(100);
        gpio_pin_set_dt(&leds[0], 0);
        k_msleep(900);
    }

    return 0;
}