#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(lora_link_test, LOG_LEVEL_DBG);

/* -----------------------------------------------------------------------
 * DTS nodelabels
 * --------------------------------------------------------------------- */

#define LED_RF1_NODE    DT_NODELABEL(led_rf_1)
#define LED_RF2_NODE    DT_NODELABEL(led_rf_2)
#define LED_RF3_NODE    DT_NODELABEL(led_rf_3)
#define LED_RF4_NODE    DT_NODELABEL(led_rf_4)
#define LED_RF5_NODE    DT_NODELABEL(led_rf_5)
#define LED_RF6_NODE    DT_NODELABEL(led_rf_6)
#define LED_RF7_NODE    DT_NODELABEL(led_rf_7)
#define LORA_NODE       DT_NODELABEL(lora0)

/* -----------------------------------------------------------------------
 * Periferiche
 * --------------------------------------------------------------------- */

static const struct device *lora_dev = DEVICE_DT_GET(LORA_NODE);

static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(LED_RF1_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_RF2_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_RF3_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_RF4_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_RF5_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_RF6_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_RF7_NODE, gpios),
};
#define NUM_LEDS ARRAY_SIZE(leds)

/* LED semantici per il test */
#define LED_TX_IDX   0   /* Lampeggia a ogni trasmissione */
#define LED_ACK_IDX  1   /* Lampeggia quando si riceve un ACK valido */
#define LED_ERR_IDX  2   /* Lampeggia in caso di timeout / errore */

/* -----------------------------------------------------------------------
 * Parametri del link test
 * --------------------------------------------------------------------- */

#define NUM_PACKETS         100
#define ACK_TIMEOUT_MS      3000   /* Tempo massimo di attesa per l'ACK */
#define INTER_PACKET_MS     300    /* Pausa tra un ciclo TX→ACK e il successivo */

/*
 * Protocollo pacchetto:
 *
 *  TX → RX:  4 byte  [0x4C, 0x54, seq_hi, seq_lo]
 *                     'L'   'T'   MSB       LSB
 *
 *  RX → TX:  4 byte  [0x41, 0x43, rssi_hi, rssi_lo]
 *                     'A'   'C'   MSB        LSB     (RSSI come int16_t big-endian)
 *
 * Il nodo ricevente deve essere programmato con la stessa configurazione radio
 * (SF8, BW250, 868 MHz, CR 4/5) e con il seguente comportamento:
 *   1. Rimane in ricezione continua (lora_recv con timeout lungo).
 *   2. All'arrivo di un pacchetto con header 'LT', legge il proprio RSSI,
 *      costruisce il pacchetto ACK e lo trasmette.
 */

/* -----------------------------------------------------------------------
 * Configurazione radio comune
 * --------------------------------------------------------------------- */

static const struct lora_modem_config lora_cfg_base = {
    .frequency    = 868000000,
    .bandwidth    = BW_250_KHZ,
    .datarate     = SF_8,
    .preamble_len = 12,
    .coding_rate  = CR_4_5,
    .tx_power     = 14,
};

static bool lora_set_mode(bool tx)
{
    struct lora_modem_config cfg = lora_cfg_base;
    cfg.tx = tx;
    int ret = lora_config(lora_dev, &cfg);
    if (ret != 0) {
        LOG_ERR("lora_config(%s) fallita: %d", tx ? "TX" : "RX", ret);
        return false;
    }
    return true;
}

/* -----------------------------------------------------------------------
 * Utility LED
 * --------------------------------------------------------------------- */

static void led_blink(int idx, int ms)
{
    if (idx < NUM_LEDS && gpio_is_ready_dt(&leds[idx])) {
        gpio_pin_set_dt(&leds[idx], 1);
        k_msleep(ms);
        gpio_pin_set_dt(&leds[idx], 0);
    }
}

/* -----------------------------------------------------------------------
 * Main
 * --------------------------------------------------------------------- */

int main(void)
{
    /* Inizializzazione LED */
    for (int i = 0; i < NUM_LEDS; i++) {
        if (gpio_is_ready_dt(&leds[i])) {
            gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
        }
    }

    /* Verifica che il modem LoRa sia pronto */
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("Modem LoRa non pronto – abort");
        return -1;
    }

    /* -----------------------------------------------------------------------
     * Strutture dati per il test
     * --------------------------------------------------------------------- */

    int16_t rssi_log[NUM_PACKETS];  /* RSSI remoto per ogni pacchetto riuscito */
    bool    pkt_ok[NUM_PACKETS];    /* true = ACK ricevuto e valido             */
    memset(rssi_log, 0, sizeof(rssi_log));
    memset(pkt_ok,   0, sizeof(pkt_ok));

    int     success_count = 0;
    int32_t rssi_sum      = 0;

    /* -----------------------------------------------------------------------
     * Inizio link test
     * --------------------------------------------------------------------- */

    LOG_INF("╔══════════════════════════════════════════════╗");
    LOG_INF("║      LoRa Link Test – %3d pacchetti          ║", NUM_PACKETS);
    LOG_INF("║  SF8 | BW 250 kHz | 14 dBm | 868 MHz        ║");
    LOG_INF("║  ACK timeout: %4d ms | inter-pkt: %3d ms   ║", ACK_TIMEOUT_MS, INTER_PACKET_MS);
    LOG_INF("╚══════════════════════════════════════════════╝");

    for (int i = 0; i < NUM_PACKETS; i++) {

        /* --- Costruzione payload TX --- */
        uint8_t tx_buf[4];
        tx_buf[0] = 'L';
        tx_buf[1] = 'T';
        tx_buf[2] = (uint8_t)((i >> 8) & 0xFF);
        tx_buf[3] = (uint8_t)(i & 0xFF);

        /* --- Trasmissione --- */
        if (!lora_set_mode(true)) {
            LOG_ERR("[%3d/%d] Impossibile entrare in TX mode", i + 1, NUM_PACKETS);
            k_msleep(INTER_PACKET_MS);
            continue;
        }

        led_blink(LED_TX_IDX, 50);

        int ret = lora_send(lora_dev, tx_buf, sizeof(tx_buf));
        if (ret < 0) {
            LOG_WRN("[%3d/%d] FAIL | TX error (%d)", i + 1, NUM_PACKETS, ret);
            led_blink(LED_ERR_IDX, 100);
            k_msleep(INTER_PACKET_MS);
            continue;
        }

        /* --- Attesa ACK --- */
        if (!lora_set_mode(false)) {
            LOG_ERR("[%3d/%d] Impossibile entrare in RX mode", i + 1, NUM_PACKETS);
            k_msleep(INTER_PACKET_MS);
            continue;
        }

        uint8_t rx_buf[8];
        int16_t rx_rssi_local;
        int8_t  rx_snr;
        int     len = lora_recv(lora_dev, rx_buf, sizeof(rx_buf),
                                K_MSEC(ACK_TIMEOUT_MS), &rx_rssi_local, &rx_snr);

        /* --- Validazione ACK --- */
        if (len == 4 && rx_buf[0] == 'A' && rx_buf[1] == 'C') {
            int16_t remote_rssi = (int16_t)(((uint16_t)rx_buf[2] << 8) | rx_buf[3]);
            rssi_log[i]    = remote_rssi;
            pkt_ok[i]      = true;
            success_count++;
            rssi_sum      += remote_rssi;

            led_blink(LED_ACK_IDX, 80);
            LOG_INF("[%3d/%d] OK   | RSSI remoto: %4d dBm  (locale: %4d dBm, SNR: %d dB)",
                    i + 1, NUM_PACKETS, remote_rssi, rx_rssi_local, rx_snr);
        } else if (len == 0 || len == -EAGAIN) {
            LOG_WRN("[%3d/%d] FAIL | Timeout ACK (%d ms)", i + 1, NUM_PACKETS, ACK_TIMEOUT_MS);
            led_blink(LED_ERR_IDX, 100);
        } else {
            LOG_WRN("[%3d/%d] FAIL | ACK malformato (len=%d)", i + 1, NUM_PACKETS, len);
            led_blink(LED_ERR_IDX, 100);
        }

        k_msleep(INTER_PACKET_MS);
    }

    /* -----------------------------------------------------------------------
     * Riepilogo finale
     * --------------------------------------------------------------------- */

    int fail_count   = NUM_PACKETS - success_count;
    /* PER in centesimi di punto percentuale per evitare float */
    int per_int      = (fail_count * 10000) / NUM_PACKETS;   /* es. 1250 = 12.50% */
    int per_whole    = per_int / 100;
    int per_frac     = per_int % 100;
    int avg_rssi     = (success_count > 0) ? (int)(rssi_sum / success_count) : 0;

    /* Pausa per svuotare completamente il buffer di log prima del riepilogo */
    k_msleep(500);

    LOG_INF(" ");
    LOG_INF("╔══════════════════════════════════════════════╗");
    LOG_INF("║             RIEPILOGO LINK TEST              ║");
    LOG_INF("╠══════════════════════════════════════════════╣");
    LOG_INF("║  Pacchetti trasmessi : %3d                   ║", NUM_PACKETS);
    LOG_INF("║  ACK ricevuti        : %3d                   ║", success_count);
    LOG_INF("║  Pacchetti persi     : %3d                   ║", fail_count);
    LOG_INF("║  Packet Error Rate   : %2d.%02d %%               ║", per_whole, per_frac);
    if (success_count > 0) {
        LOG_INF("║  RSSI medio (remoto) : %4d dBm              ║", avg_rssi);
    } else {
        LOG_INF("║  RSSI medio (remoto) : N/A                   ║");
    }
    LOG_INF("╠══════════════════════════════════════════════╣");
    LOG_INF("║  Dettaglio per pacchetto:                    ║");

    /* Delay tra ogni riga per non saturare il buffer del logger Zephyr */
    for (int i = 0; i < NUM_PACKETS; i++) {
        if (pkt_ok[i]) {
            LOG_INF("║  PKT %3d : OK   | RSSI %4d dBm             ║",
                    i + 1, rssi_log[i]);
        } else {
            LOG_INF("║  PKT %3d : FAIL                              ║", i + 1);
        }
        k_msleep(20);
    }

    LOG_INF("╚══════════════════════════════════════════════╝");

    /* Loop vuoto: il test è terminato */
    while (1) {
        k_msleep(5000);
    }

    return 0;
}