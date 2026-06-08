/*
 * wio_test/src/main.c
 *
 * App di test minimale per XIAO ESP32-S3 + Wio-SX1262 (connettore B2B).
 *
 * Comportamento:
 *   - All'avvio stampa i GPIO assegnati a LED e bottone (debug overlay).
 *   - Il LED lampeggia a 1 Hz per indicare che il firmware è vivo.
 *   - Il bottone usa un interrupt GPIO (edge falling, active LOW).
 *     Alla pressione: il LED lampeggia 5 volte velocemente + TX LoRa.
 *   - La radio viene inizializzata; se fallisce il LED lampeggia in
 *     codice Morse SOS (· · · — — — · · ·) in loop.
 *
 * Alias dall'overlay:
 *   led-0  → led_wio  (GPIO48, active HIGH)
 *   sw0    → user_button (GPIO21, active LOW)
 *   lora0  → lora_sx1262
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wio_test, LOG_LEVEL_DBG);

/* ── Device Tree ────────────────────────────────────────────────── */
#define LED_NODE    DT_ALIAS(led_0)
#define BUTTON_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec led    = GPIO_DT_SPEC_GET(LED_NODE,    gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

/* ── LoRa ───────────────────────────────────────────────────────── */
#define LORA_FREQUENCY    868100000  /* 868.1 MHz – EU868 ch0 */
#define LORA_TX_POWER     14         /* dBm                    */

static const uint8_t tx_payload[] = "XIAO+WIO TEST";

/* ── Interrupt bottone ──────────────────────────────────────────── */
static struct gpio_callback button_cb_data;
static volatile bool        button_pressed;

static void button_isr(const struct device *dev,
                       struct gpio_callback *cb,
                       uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    button_pressed = true;
}

/* ── Helpers LED ─────────────────────────────────────────────────── */
static void led_blink(int times, int period_ms)
{
    for (int i = 0; i < times; i++) {
        gpio_pin_set_dt(&led, 1);
        k_msleep(period_ms / 2);
        gpio_pin_set_dt(&led, 0);
        k_msleep(period_ms / 2);
    }
}

static void led_sos(void)
{
    /* S = · · ·   O = — — —   S = · · · */
    const int dot  = 150;
    const int dash = 450;
    const int gap  = 150;
    const int word = 600;
    int pattern[] = { dot, dot, dot, dash, dash, dash, dot, dot, dot };
    for (int i = 0; i < 9; i++) {
        gpio_pin_set_dt(&led, 1);
        k_msleep(pattern[i]);
        gpio_pin_set_dt(&led, 0);
        k_msleep(gap);
    }
    k_msleep(word);
}

/* ── TX LoRa ─────────────────────────────────────────────────────── */
static int lora_send_test(const struct device *lora_dev)
{
    int ret = lora_send(lora_dev,
                        (uint8_t *)tx_payload,
                        sizeof(tx_payload) - 1);
    if (ret < 0) {
        LOG_ERR("lora_send() fallito: %d", ret);
    } else {
        LOG_INF("TX OK: \"%s\"", tx_payload);
    }
    return ret;
}

/* ── main ────────────────────────────────────────────────────────── */
int main(void)
{
    int ret;

    LOG_INF("=== XIAO ESP32-S3 + Wio-SX1262 — avvio ===");

    /* ── 1. LED ──────────────────────────────────────────────────── */
    if (!gpio_is_ready_dt(&led)) {
        LOG_ERR("LED GPIO controller non pronto");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Configurazione LED fallita: %d", ret);
        return ret;
    }
    /* Stampa il GPIO reale assegnato — utile per verificare l'overlay */
    LOG_INF("LED  : GPIO%d, active-%s",
            led.pin,
            (led.dt_flags & GPIO_ACTIVE_LOW) ? "LOW" : "HIGH");

    /* Lampeggio di avvio per confermare che il LED funziona */
    led_blink(3, 200);

    /* ── 2. Bottone (interrupt) ───────────────────────────────────── */
    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("Button GPIO controller non pronto");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Configurazione bottone fallita: %d", ret);
        return ret;
    }

    /* Configura interrupt su fronte di discesa (pressione, active LOW) */
    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Configurazione interrupt bottone fallita: %d", ret);
        return ret;
    }

    gpio_init_callback(&button_cb_data, button_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    LOG_INF("BTN  : GPIO%d, active-%s — interrupt abilitato",
            button.pin,
            (button.dt_flags & GPIO_ACTIVE_LOW) ? "LOW" : "HIGH");

    /* ── 3. LoRa ─────────────────────────────────────────────────── */
    k_msleep(100);  /* lascia stabilizzare l'SX1262 dopo il boot */
    const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

    if (!device_is_ready(lora_dev)) {
        LOG_ERR("SX1262 non pronto — controlla SPI/GPIO nell'overlay");
        while (1) {
            led_sos();
        }
    }
    LOG_INF("LoRa : SX1262 pronto");

    struct lora_modem_config lora_cfg = {
        .frequency    = LORA_FREQUENCY,
        .bandwidth    = BW_125_KHZ,
        .datarate     = SF_7,
        .preamble_len = 8,
        .coding_rate  = CR_4_5,
        .tx_power     = LORA_TX_POWER,
        .tx           = true,
    };

    ret = lora_config(lora_dev, &lora_cfg);
    if (ret < 0) {
        LOG_ERR("lora_config() fallito: %d", ret);
        while (1) {
            led_sos();
        }
    }
    LOG_INF("LoRa : configurato @ %d MHz SF7 BW125 CR4/5 %d dBm",
            LORA_FREQUENCY / 1000000, LORA_TX_POWER);

    /* TX iniziale di verifica */
    ret = lora_send_test(lora_dev);
    if (ret == 0) {
        led_blink(2, 400);
    }

    LOG_INF("=== Loop principale — premi il bottone per TX ===");

    /* ── Loop principale ─────────────────────────────────────────── */
    while (1) {
        /* Lampeggio lento 1 Hz — firmware vivo */
        gpio_pin_toggle_dt(&led);
        k_msleep(500);

        if (button_pressed) {
            button_pressed = false;
            LOG_INF("Bottone premuto — TX LoRa...");
            gpio_pin_set_dt(&led, 0);
            ret = lora_send_test(lora_dev);
            led_blink(5, 100);  /* feedback visivo */
        }
    }

    return 0;
}