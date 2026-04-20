#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lora_tx, LOG_LEVEL_DBG);

/* 1. Binding Hardware a tempo di compilazione */
static const struct device *lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora_sx1261));

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
