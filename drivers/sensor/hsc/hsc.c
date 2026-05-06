/* Honeywell HSCx Series Pressure sensors
 * For SPI interface development the Technical note [SPI Communications with Honeywell Digital Output
Pressure Sensors](https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/en-us/products/sensors/pressure-sensors/common/documents/sps-siot-spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-ciid-45843.pdf)
 * was used.
 * This library should be valid for all Honeywell sensors with SPI interface.
 * 
 * All sensors are programmed to output 4 bytes of data but not all 4 might be valid
 * You don't need to send any message to it.
 * 
 * Keep in mind that this sensor is quite  "slow" as its maximum sck frequency is of 800kHz
*/

#include "hsc.h"

//
#include <stdbool.h>

// Zephyr LIBs
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT honeywell_hsc

LOG_MODULE_REGISTER(hsc, CONFIG_SENSOR_LOG_LEVEL);

struct hsc_sample
{
    int16_t pressure;
    int16_t temperature;
};

struct hsc_data
{
    bool status0;
    bool status1;

    struct hsc_sample last_sample;
};

struct hsc_config
{
    struct spi_dt_spec spi;

    // Parametri di calibrazione dal devicetree
    int32_t p_min;
    int32_t p_max;

    //Fattori di scala calcolati in init
    int32_t p_scale_micro;  /* Moltiplicatore pressione in micro-unità */
    int64_t p_offset_micro; /* Offset pressione in micro-unità */
};

int hsc_check_status(uint8_t * buffer) {
    uint8_t status = (rx_buffer[0] >> 6) & 0x03; //Prendo solo il primo byte, schifto e maschero

    switch (status) {
        case 0x00: /* Normal operation, valid data */
            break;
        case 0x01: /* Device in command mode */
            LOG_ERR("Sensore in command mode");
            return -EBUSY;
        case 0x02: /* Stale data */
            LOG_DBG("Dati vecchi letti dal buffer");
            return -EAGAIN;
        case 0x03: /* Diagnostic condition (errore) */
            LOG_ERR("Errore diagnostico del sensore");
            return -EIO;
        default:
            return -EINVAL;
    }
}


static int hsc_sample_fetch(const struct device * dev, enum sensor_channel chan) {
    CHECKIF(dev == NULL) { return -EINVAL; }

    struct hsc_data *data = (struct hsc_data *)dev->data;
    struct hsc_config *conf = (struct hsc_config *)dev->config;

    uint8_t rx_buf[4] = {0};

    const struct spi_buf rx = {.buf = (void *)rx_buf, .len = sizeof(rx_buf)};
    const struct spi_buf_set rx_bufs {.buffers = &rx, .count = 1};

    int ret = spi_receive_dt(conf->spi, &rx_bufs); //NOTA: lettura bloccante
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data: %d (%s)", ret, strerror(-ret));
    }

    hsc_check_status(rx_buf); //TODO: qua viene ritornato un codice errore, ma non viene controllato

    // Estrazione dei conteggi di pressione e temperatura

    data->last_sample->pressure = ((rx_buffer[0] & 0x3F) << 8) | rx_buffer[1];
    data->last_sample->temperature = (rx_buffer[2] << 3) | (rx_buffer[3] >> 5);

    return 0;
}


static int hsc_init(const struct device * dev) {
    CHECKIF(dev == NULL) { return -EINVAL; }


    struct hsc_data *data = (struct hsc_data *)dev->data;
    struct hsc_config *conf = (struct hsc_config *)dev->config;

    LOG_DBG("Initializing HSC sensor");

    if (!spi_is_ready_dt(&conf->spi)) {
        LOG_ERR("SPI device not ready");
    return -1;
    }

    /* PRECALCOLO DELLA PRESSIONE */
    /* Moltiplichiamo per 10^9 (nano) per mantenere un'altissima precisione dopo la divisione */
    int64_t p_range = conf->p_max - conf->p_min;
    int64_t out_range = conf->out_max - conf->out_min;
    
    /* Salviamo un int32_t per la massima velocità a runtime */
    conf->p_scale_micro = (int32_t)((p_range * 1000000LL) / out_range);
    conf->p_offset_micro = (int64_t)conf->p_min * 1000000LL;
    return 0;
}

int hsc_channel_get(const struct device *dev,
                        enum sensor_channel chan,
                        struct sensor_value *val) {
    const struct hsc_data *data = dev->data;
    struct hsc_conf *config = dev->config;

    switch (chan) {
    case SENSOR_CHAN_PRESS:
        /*
         * Formula Pressione:
         * Pressure = [(Output - Output_min) * (Pressure_max - Pressure_min) / 
         * (Output_max - Output_min)] + Pressure_min [cite: 118, 121]
         */
        /* 1. Sottrazione del limite inferiore grezzo. 
         * Il risultato massimo è ~16384, comodamente gestibile a 32-bit.
         */
        int32_t output_diff = (int32_t)data->raw_pressure - config->out_min;

        /* 2. Calcolo della pressione in micro-unità (es. micro-Pascal).
         * Il cast a (int64_t) su output_diff forza la moltiplicazione a 64-bit.
         * Questo è FONDAMENTALE per evitare overflow qualora p_offset_micro 
         * contenga valori molto grandi (es. pressioni atmosferiche in Pascal).
         */
        int64_t press_micro = ((int64_t)output_diff * config->p_scale_micro) + config->p_offset_micro;
        
        /* 3. Estrazione della parte intera (val1) e frazionaria (val2) in milionesimi */
        val->val1 = (int32_t)(press_micro / 1000000LL);
        val->val2 = (int32_t)(press_micro % 1000000LL);

        /* 4. Correzione Zephyr per i numeri negativi:
         * Le API richiedono che val2 (la parte decimale) sia sempre positivo.
         * Se il numero è negativo e ha dei decimali, "spostiamo" un intero sui decimali.
         */
        if (press_micro < 0 && val->val2 != 0) {
            val->val1 -= 1;
            val->val2 += 1000000;
        }
        //TODO: questa procedura l'ho fatta con gemini, è possibile che non sia la cosa più ottimizzata
        break;

    case SENSOR_CHAN_GAUGE_TEMP:
        /* 1. Calcolo della temperatura usando esclusivamente registri a 32-bit.
         * Il range del sensore garantisce che la moltiplicazione non vada mai in overflow.
         * È estremamente veloce e occupa pochi cicli di clock.
         */
        int32_t temp_micro = ((int32_t)data->raw_temp * HSC_TEMP_SCALE_MICRO) + HSC_TEMP_OFFSET_MICRO;

        /* 2. Estrazione per Zephyr */
        val->val1 = temp_micro / 1000000;
        val->val2 = temp_micro % 1000000;

        /* 3. Correzione numeri negativi */
        if (temp_micro < 0 && val->val2 != 0) {
            val->val1 -= 1;
            val->val2 += 1000000;
        }
        break;
    
    default:
        return -ENOTSUP;
    }

    return 0;
}

// questa la vuole zephyr
int hsc_attr_set (const struct device *dev, enum sensor_channel chan,
                    enum sensor_attribute attr,
                    const struct sensor_value *val) {
    return -1
}


///////////
// macro, metodi e struct base di Zephyr //
static DEVICE_API(sensor, hsc_driver_api) = {
    .sample_fetch = hsc_sample_fetch,
    .channel_get = hsc_channel_get,
    .attr_set = hsc_attr_set,
};

#define HSC_INIT(i)                                                             \
    static struct hsc_data hsc_data_##i = {0};                                  \
    static struct hsc_config hsc_config_##i = {                                 \
        .spi = SPI_DT_SPEC_INST_GET(i, SPI_WORD_SET(8) | SPI_TRANSFER_MSB),     \
    };                                                                          \
    SENSOR_DEVICE_DT_INST_DEFINE(i, hsc_init, NULL, &hsc_data_##i, &hsc_config_##i, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &hsc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HSC_INIT)