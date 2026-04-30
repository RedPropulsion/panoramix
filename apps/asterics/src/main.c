#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/regulator.h>

LOG_MODULE_REGISTER(hw_debug_test, LOG_LEVEL_INF);

/* Costanti dei registri e valori attesi */
#define BMI088_ACCEL_CHIP_ID_REG 0x00
#define BMI088_ACCEL_CHIP_ID_VAL 0x1E
#define BMI088_GYRO_CHIP_ID_REG  0x00
#define BMI088_GYRO_CHIP_ID_VAL  0x0F
#define ICM42688_WHO_AM_I_REG    0x75
#define ICM42688_WHO_AM_I_VAL    0x47
#define H3LIS331DL_WHO_AM_I_REG  0x0F
#define H3LIS331DL_WHO_AM_I_VAL  0x32
#define LIS2MDL_WHO_AM_I_REG     0x4F
#define LIS2MDL_WHO_AM_I_VAL     0x40
#define BNO055_CHIP_ID_REG       0x00
#define BNO055_CHIP_ID_VAL       0xA0
#define MS5611_CMD_PROM_READ     0xA0 /* Comando base per lettura PROM (coefficiente 0) */

/* ---------------------------------------------------------------------------
 * Helper: lettura registro SPI con byte di indirizzo (MSB=1 → read)
 * dummy_byte: alcuni device (es. BMI088 accel) richiedono un byte di turnaround
 *             tra il comando e il dato → rx_buffer è [dummy_cmd][dummy][dato]
 * --------------------------------------------------------------------------- */
static int read_spi_reg(const struct spi_dt_spec *spec, uint8_t reg,
                        uint8_t *val, bool dummy_byte)
{
    if (!spi_is_ready_dt(spec)) {
        LOG_ERR("SPI bus non pronto per %s", spec->bus->name);
        return -ENODEV;
    }

    /* TX: reg | 0x80 (read bit), poi eventuali byte dummy */
    uint8_t tx_buffer[3] = { reg | 0x80, 0x00, 0x00 };
    struct spi_buf tx_buf  = { .buf = tx_buffer, .len = dummy_byte ? 3 : 2 };
    struct spi_buf_set tx  = { .buffers = &tx_buf, .count = 1 };

    uint8_t rx_buffer[3] = {0};
    struct spi_buf rx_b   = { .buf = rx_buffer, .len = dummy_byte ? 3 : 2 };
    struct spi_buf_set rx = { .buffers = &rx_b, .count = 1 };

    int ret = spi_transceive_dt(spec, &tx, &rx);
    if (ret == 0) {
        *val = dummy_byte ? rx_buffer[2] : rx_buffer[1];
    }
    return ret;
}

/* ---------------------------------------------------------------------------
 * Helper MS5611: legge il primo coefficiente PROM (0xA0).
 * Il trasferimento è un singolo burst di 3 byte (CS abbassato per tutta la
 * durata): [CMD][DATA_MSB][DATA_LSB].  TX e RX hanno la stessa lunghezza
 * per evitare comportamenti dipendenti dal padding del driver.
 * Un valore PROM diverso da 0x0000 e 0xFFFF indica chip presente e funzionante.
 * --------------------------------------------------------------------------- */
static int test_ms5611(const struct spi_dt_spec *spec, const char *name)
{
    if (!spi_is_ready_dt(spec)) {
        LOG_ERR("[FAIL] %s -> Bus non pronto", name);
        return -ENODEV;
    }

    /* 3 byte: comando + 2 dummy (il chip risponde nelle ultime due posizioni) */
    uint8_t tx_buffer[3] = { MS5611_CMD_PROM_READ, 0x00, 0x00 };
    struct spi_buf tx_buf  = { .buf = tx_buffer, .len = 3 };
    struct spi_buf_set tx  = { .buffers = &tx_buf, .count = 1 };

    uint8_t rx_buffer[3] = {0};
    struct spi_buf rx_b   = { .buf = rx_buffer, .len = 3 };
    struct spi_buf_set rx = { .buffers = &rx_b, .count = 1 };

    int ret = spi_transceive_dt(spec, &tx, &rx);
    if (ret != 0) {
        LOG_ERR("[FAIL] %s -> Errore SPI (%d)", name, ret);
        return ret;
    }

    uint16_t prom_val = ((uint16_t)rx_buffer[1] << 8) | rx_buffer[2];
    if (prom_val != 0x0000 && prom_val != 0xFFFF) {
        LOG_INF("[PASS] %s -> PROM[0] letta: 0x%04X", name, prom_val);
        return 0;
    }

    LOG_ERR("[FAIL] %s -> Valore PROM anomalo: 0x%04X (bus staccato?)", name, prom_val);
    return -EIO;
}

/* --------------------------------------------------------------------------- */
static void validate_id(const char *name, int ret, uint8_t read_val,
                        uint8_t expected_val)
{
    if (ret < 0) {
        LOG_ERR("[FAIL] %s -> Errore bus (%d)", name, ret);
    } else if (read_val == expected_val) {
        LOG_INF("[PASS] %s -> ID: 0x%02X (atteso: 0x%02X)", name,
                read_val, expected_val);
    } else {
        LOG_WRN("[FAIL] %s -> ID mismatch! Letto: 0x%02X, Atteso: 0x%02X",
                name, read_val, expected_val);
    }
}

/* ===========================================================================
 * NOTA su ms5611_a / ms5611_b
 * ---------------------------------------------------------------------------
 * Il binding "ams,ms5611" non eredita da spi-device.yaml, quindi il DT
 * generator NON produce le macro:
 *   _P_duplex, _P_frame_format, _P_spi_interframe_delay_ns
 * che SPI_CONFIG_DT (usato da SPI_DT_SPEC_GET) richiede a compile-time.
 *
 * Soluzione: costruire le spi_dt_spec a mano prelevando il bus e il pin CS
 * direttamente dal DT del controller SPI, esattamente come già fatto per i
 * device su SPI4. Non richiede alcuna modifica al DTS.
 *
 * Mappa CS-gpios (dal DTS):
 *   SPI2: index 0 = MS5611_A, 1 = gyro_A, 2 = accel_A, 3 = icm_A
 *   SPI3: index 0 = MS5611_B, 1 = gyro_B, 2 = accel_B, 3 = icm_B
 * =========================================================================== */

int main(void)
{
    k_msleep(500); /* Attendi stabilizzazione tensioni */

    const struct device *reg = DEVICE_DT_GET(DT_NODELABEL(sensor_board_vdd));
    if (device_is_ready(reg)) {
        regulator_enable(reg);
        k_msleep(10); /* lascia stabilizzare */
        LOG_INF("sensor_board_vdd abilitato");
    } else {
        LOG_ERR("Regolatore non pronto! Controlla CONFIG_REGULATOR=y");
    }

    LOG_INF("=== Avvio Test Debug Hardware SPI/I2C ===");

    uint8_t val;
    int ret;

    /* -------------------------------------------------------------------------
     * SPI2 (Blocco A)
     * BMI088 e ICM42688 usano SPI_DT_SPEC_GET (binding completi).
     * MS5611 usa struct manuale (vedi nota sopra).
     * ------------------------------------------------------------------------- */
    const struct spi_dt_spec bmi088_accel_a =
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_accel_a), SPI_WORD_SET(8), 0);
    const struct spi_dt_spec bmi088_gyro_a =
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_gyro_a), SPI_WORD_SET(8), 0);
    const struct spi_dt_spec icm42688_a =
        SPI_DT_SPEC_GET(DT_NODELABEL(icm42688p_a), SPI_WORD_SET(8), 0);

    /* MS5611_A: costruzione manuale (CS index 0 su SPI2) */
    const struct device *spi2_bus = DEVICE_DT_GET(DT_NODELABEL(spi2));
    struct spi_dt_spec ms5611_a = {
        .bus = spi2_bus,
        .config = {
            .frequency = 1000000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 0,
            .cs = {
                .gpio  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi2), cs_gpios, 0),
                .delay = 0,
            },
        },
    };

    /* BMI088 accel richiede 1 byte di turnaround (dummy_byte = true) */
    ret = read_spi_reg(&bmi088_accel_a, BMI088_ACCEL_CHIP_ID_REG, &val, true);
    validate_id("BMI088_Accel_A (SPI2)", ret, val, BMI088_ACCEL_CHIP_ID_VAL);

    ret = read_spi_reg(&bmi088_gyro_a, BMI088_GYRO_CHIP_ID_REG, &val, false);
    validate_id("BMI088_Gyro_A  (SPI2)", ret, val, BMI088_GYRO_CHIP_ID_VAL);

    ret = read_spi_reg(&icm42688_a, ICM42688_WHO_AM_I_REG, &val, false);
    validate_id("ICM42688P_A    (SPI2)", ret, val, ICM42688_WHO_AM_I_VAL);

    test_ms5611(&ms5611_a, "MS5611_A       (SPI2)");

    /* -------------------------------------------------------------------------
     * SPI3 (Blocco B)
     * Stessa strategia: SPI_DT_SPEC_GET per BMI088/ICM, manuale per MS5611.
     * ------------------------------------------------------------------------- */
    const struct spi_dt_spec bmi088_accel_b =
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_accel_b), SPI_WORD_SET(8), 0);
    const struct spi_dt_spec bmi088_gyro_b =
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_gyro_b), SPI_WORD_SET(8), 0);
    const struct spi_dt_spec icm42688_b =
        SPI_DT_SPEC_GET(DT_NODELABEL(icm42688p_b), SPI_WORD_SET(8), 0);

    /* MS5611_B: costruzione manuale (CS index 0 su SPI3) */
    const struct device *spi3_bus = DEVICE_DT_GET(DT_NODELABEL(spi3));
    struct spi_dt_spec ms5611_b = {
        .bus = spi3_bus,
        .config = {
            .frequency = 1000000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 0,
            .cs = {
                .gpio  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi3), cs_gpios, 0),
                .delay = 0,
            },
        },
    };

    ret = read_spi_reg(&bmi088_accel_b, BMI088_ACCEL_CHIP_ID_REG, &val, true);
    validate_id("BMI088_Accel_B (SPI3)", ret, val, BMI088_ACCEL_CHIP_ID_VAL);

    ret = read_spi_reg(&bmi088_gyro_b, BMI088_GYRO_CHIP_ID_REG, &val, false);
    validate_id("BMI088_Gyro_B  (SPI3)", ret, val, BMI088_GYRO_CHIP_ID_VAL);

    ret = read_spi_reg(&icm42688_b, ICM42688_WHO_AM_I_REG, &val, false);
    validate_id("ICM42688P_B    (SPI3)", ret, val, ICM42688_WHO_AM_I_VAL);

    test_ms5611(&ms5611_b, "MS5611_B       (SPI3)");

    /* -------------------------------------------------------------------------
     * SPI4 (Ausiliari): H3LIS331DL, LIS2MDL, HSC diff
     * Tutti costruiti a mano (binding con proprietà mancanti o parziali).
     * Mappa CS-gpios (dal DTS):
     *   index 0 = BMP585 (disabled), 1 = H3LIS331DL, 2 = LIS2MDL, 3 = HSC
     * ------------------------------------------------------------------------- */
    const struct device *spi4_bus = DEVICE_DT_GET(DT_NODELABEL(spi4));

    struct spi_dt_spec h3lis331dl = {
        .bus = spi4_bus,
        .config = {
            .frequency = 1000000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 1,
            .cs = {
                .gpio  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi4), cs_gpios, 1),
                .delay = 0,
            },
        },
    };

    struct spi_dt_spec lis2mdl = {
        .bus = spi4_bus,
        .config = {
            .frequency = 1000000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 2,
            .cs = {
                .gpio  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi4), cs_gpios, 2),
                .delay = 0,
            },
        },
    };

    struct spi_dt_spec hsc_diff = {
        .bus = spi4_bus,
        .config = {
            .frequency = 800000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 3,
            .cs = {
                .gpio  = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi4), cs_gpios, 3),
                .delay = 0,
            },
        },
    };

    ret = read_spi_reg(&h3lis331dl, H3LIS331DL_WHO_AM_I_REG, &val, false);
    validate_id("H3LIS331DL     (SPI4)", ret, val, H3LIS331DL_WHO_AM_I_VAL);

    ret = read_spi_reg(&lis2mdl, LIS2MDL_WHO_AM_I_REG, &val, false);
    validate_id("LIS2MDL        (SPI4)", ret, val, LIS2MDL_WHO_AM_I_VAL);

    /* HSC: device read-only, non ha registri — leggiamo direttamente 2 byte */
    if (spi_is_ready_dt(&hsc_diff)) {
        uint8_t rx_hsc[2] = {0};
        struct spi_buf rx_b_hsc    = { .buf = rx_hsc, .len = 2 };
        struct spi_buf_set rx_hsc_set = { .buffers = &rx_b_hsc, .count = 1 };

        ret = spi_read_dt(&hsc_diff, &rx_hsc_set);
        if (ret == 0) {
            /* Bit [15:14] = status (00 = normal), [13:0] = pressione raw */
            uint8_t status = (rx_hsc[0] >> 6) & 0x03;
            LOG_INF("[PASS] HSC_DIFF       (SPI4) -> raw: 0x%02X 0x%02X (status=%d)",
                    rx_hsc[0], rx_hsc[1], status);
        } else {
            LOG_ERR("[FAIL] HSC_DIFF       (SPI4) -> Errore lettura (%d)", ret);
        }
    } else {
        LOG_ERR("[FAIL] HSC_DIFF       (SPI4) -> Bus non pronto");
    }

    /* -------------------------------------------------------------------------
     * I2C4: BNO055
     * ------------------------------------------------------------------------- */
    const struct i2c_dt_spec bno055 = I2C_DT_SPEC_GET(DT_NODELABEL(bno055));
    if (i2c_is_ready_dt(&bno055)) {
        ret = i2c_reg_read_byte_dt(&bno055, BNO055_CHIP_ID_REG, &val);
        validate_id("BNO055         (I2C4)", ret, val, BNO055_CHIP_ID_VAL);
    } else {
        LOG_ERR("[FAIL] BNO055         (I2C4) -> Bus non pronto");
    }

    LOG_INF("=== Test completato ===");

    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}