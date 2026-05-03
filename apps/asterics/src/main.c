#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

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
#define MS5611_CMD_PROM_READ     0xA0

/* ===========================================================================
 * INIT CS GPIO — funzione critica
 * ---------------------------------------------------------------------------
 * Su STM32H7 tutti i GPIO si resettano in modalità ANALOG (MODER=11).
 * Il driver SPI usa gpio_pin_set_dt() per togglare CS, ma questa funzione
 * richiede che il pin sia già configurato come OUTPUT — altrimenti non ha
 * effetto e CS resta flottante.
 *
 * Il problema si manifesta anche per i device con driver Zephyr (bmi08x,
 * icm42688): quei driver configurano il CS nel loro init(), ma con
 * regulator-boot-on l'ordine di init non è garantito. Se i driver SPI
 * si inizializzano prima che sensor_board_vdd sia attivo, l'init fallisce
 * silenziosamente e il GPIO CS non viene mai portato in modalità OUTPUT.
 *
 * Soluzione: configurare esplicitamente TUTTI i CS come GPIO_OUTPUT_INACTIVE
 * prima di qualsiasi transazione SPI.
 * GPIO_OUTPUT_INACTIVE + GPIO_ACTIVE_LOW (dal DTS) = pin fisico HIGH = CS off.
 *
 * Mappa CS (dal DTS):
 *   SPI2:  idx0=MS5611_A  idx1=gyro_A   idx2=accel_A  idx3=icm_A
 *   SPI3:  idx0=MS5611_B  idx1=gyro_B   idx2=accel_B  idx3=icm_B
 *   SPI4:  idx0=BMP585    idx1=H3LIS    idx2=LIS2MDL  idx3=HSC
 * =========================================================================== */
static int cs_gpios_init(void)
{
    int ret = 0;

#define CS_INIT(bus_label, idx)                                                    \
    do {                                                                           \
        const struct gpio_dt_spec _cs =                                            \
            GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(bus_label), cs_gpios, idx);      \
        int _r = gpio_pin_configure_dt(&_cs, GPIO_OUTPUT_INACTIVE);               \
        if (_r < 0) {                                                              \
            LOG_ERR("CS init FAIL " #bus_label "[%d] err=%d", idx, _r);           \
            ret = _r;                                                              \
        }                                                                          \
    } while (0)

    CS_INIT(spi2, 0); /* MS5611_A  — gpiog 2 */
    CS_INIT(spi2, 1); /* gyro_A    — gpiog 3 */
    CS_INIT(spi2, 2); /* accel_A   — gpiog 4 */
    CS_INIT(spi2, 3); /* icm_A     — gpiog 5 */

    CS_INIT(spi3, 0); /* MS5611_B  — gpiod 2 */
    CS_INIT(spi3, 1); /* gyro_B    — gpiod 4 */
    CS_INIT(spi3, 2); /* accel_B   — gpiod 3 */
    CS_INIT(spi3, 3); /* icm_B     — gpiod 5 */

    CS_INIT(spi4, 0); /* BMP585    — gpiog 7 (disabled, ma CS va comunque deasserted) */
    CS_INIT(spi4, 1); /* H3LIS331  — gpiog 8 */
    CS_INIT(spi4, 2); /* LIS2MDL   — gpiod 10 */
    CS_INIT(spi4, 3); /* HSC_DIFF  — gpiog 6 */

#undef CS_INIT
    return ret;
}

/* ---------------------------------------------------------------------------
 * Helper: lettura registro SPI (bit MSB=1 → operazione di read)
 * dummy_byte: il BMI088 accel richiede un byte di turnaround tra cmd e dato
 * --------------------------------------------------------------------------- */
static int read_spi_reg(const struct spi_dt_spec *spec, uint8_t reg,
                        uint8_t *val, bool dummy_byte)
{
    if (!spi_is_ready_dt(spec)) {
        LOG_ERR("SPI bus non pronto: %s", spec->bus->name);
        return -ENODEV;
    }

    uint8_t tx_buf[3] = { reg | 0x80, 0x00, 0x00 };
    uint8_t rx_buf[3] = { 0 };
    struct spi_buf     tx_b   = { .buf = tx_buf, .len = dummy_byte ? 3 : 2 };
    struct spi_buf     rx_b   = { .buf = rx_buf, .len = dummy_byte ? 3 : 2 };
    struct spi_buf_set tx_set = { .buffers = &tx_b, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_b, .count = 1 };

    int ret = spi_transceive_dt(spec, &tx_set, &rx_set);
    if (ret == 0) {
        *val = dummy_byte ? rx_buf[2] : rx_buf[1];
    }
    return ret;
}

/* ---------------------------------------------------------------------------
 * Helper MS5611: lettura primo word PROM (cmd 0xA0).
 * Burst 3 byte: [CMD][MSB][LSB]. Valido se != 0x0000 e != 0xFFFF.
 * --------------------------------------------------------------------------- */
static int test_ms5611(const struct spi_dt_spec *spec, const char *name)
{
    if (!spi_is_ready_dt(spec)) {
        LOG_ERR("[FAIL] %s -> Bus non pronto", name);
        return -ENODEV;
    }

    uint8_t tx_buf[3] = { MS5611_CMD_PROM_READ, 0x00, 0x00 };
    uint8_t rx_buf[3] = { 0 };
    struct spi_buf     tx_b   = { .buf = tx_buf, .len = 3 };
    struct spi_buf     rx_b   = { .buf = rx_buf, .len = 3 };
    struct spi_buf_set tx_set = { .buffers = &tx_b, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_b, .count = 1 };

    int ret = spi_transceive_dt(spec, &tx_set, &rx_set);
    if (ret != 0) {
        LOG_ERR("[FAIL] %s -> Errore SPI (%d)", name, ret);
        return ret;
    }

    uint16_t prom = ((uint16_t)rx_buf[1] << 8) | rx_buf[2];
    if (prom != 0x0000 && prom != 0xFFFF) {
        LOG_INF("[PASS] %s -> PROM[0]: 0x%04X", name, prom);
        return 0;
    }

    LOG_ERR("[FAIL] %s -> PROM anomala: 0x%04X", name, prom);
    return -EIO;
}

/* --------------------------------------------------------------------------- */
static void validate_id(const char *name, int ret, uint8_t got, uint8_t exp)
{
    if (ret < 0) {
        LOG_ERR("[FAIL] %s -> Errore bus (%d)", name, ret);
    } else if (got == exp) {
        LOG_INF("[PASS] %s -> ID: 0x%02X", name, got);
    } else {
        LOG_WRN("[FAIL] %s -> ID mismatch! got=0x%02X exp=0x%02X", name, got, exp);
    }
}

/* =========================================================================== */
int main(void)
{
    /* -----------------------------------------------------------------------
     * 1. Regolatore sensori
     *    regulator-boot-on lo attiva automaticamente, ma la chiamata esplicita
     *    garantisce che sia up prima di cs_gpios_init() e dei test.
     * ----------------------------------------------------------------------- */
    k_msleep(100);

    const struct device *reg = DEVICE_DT_GET(DT_NODELABEL(sensor_board_vdd));
    if (device_is_ready(reg)) {
        int r = regulator_enable(reg);
        if (r < 0 && r != -EALREADY) {
            LOG_ERR("regulator_enable() fallito: %d", r);
        } else {
            LOG_INF("sensor_board_vdd: ON");
        }
    } else {
        LOG_ERR("Regolatore non pronto! CONFIG_REGULATOR=y in prj.conf?");
    }

    /* -----------------------------------------------------------------------
     * 2. Init CS GPIO
     *    Configura tutti i CS come output-inactive PRIMA di qualsiasi
     *    transazione SPI. Vedi commento a cs_gpios_init().
     * ----------------------------------------------------------------------- */
    if (cs_gpios_init() < 0) {
        LOG_WRN("Uno o più CS GPIO non inizializzati correttamente");
    }

    /* -----------------------------------------------------------------------
     * 3. Attesa power-up sensori
     *    BMI088 gyro: ~30 ms da power-on (domina). Usiamo 50 ms con margine.
     * ----------------------------------------------------------------------- */
    k_msleep(50);
    LOG_INF("=== Avvio Test Debug Hardware SPI/I2C ===");

    uint8_t val;
    int ret;

    /* -----------------------------------------------------------------------
     * SPI2 — Blocco A
     * BMI088/ICM: SPI_DT_SPEC_GET (binding completi).
     * MS5611: struct manuale (ams,ms5611 non eredita spi-device.yaml).
     * ----------------------------------------------------------------------- */
    const struct spi_dt_spec bmi088_accel_a =
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_accel_a), SPI_WORD_SET(8), 0);
    const struct spi_dt_spec bmi088_gyro_a =
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_gyro_a), SPI_WORD_SET(8), 0);
    const struct spi_dt_spec icm42688_a =
        SPI_DT_SPEC_GET(DT_NODELABEL(icm42688p_a), SPI_WORD_SET(8), 0);

    const struct device *spi2_bus = DEVICE_DT_GET(DT_NODELABEL(spi2));
    struct spi_dt_spec ms5611_a = {
        .bus = spi2_bus,
        .config = {
            .frequency = 1000000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 0,
            .cs = { .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi2), cs_gpios, 0), .delay = 0 },
        },
    };

    ret = read_spi_reg(&bmi088_accel_a, BMI088_ACCEL_CHIP_ID_REG, &val, true);
    validate_id("BMI088_Accel_A (SPI2)", ret, val, BMI088_ACCEL_CHIP_ID_VAL);

    ret = read_spi_reg(&bmi088_gyro_a, BMI088_GYRO_CHIP_ID_REG, &val, false);
    validate_id("BMI088_Gyro_A  (SPI2)", ret, val, BMI088_GYRO_CHIP_ID_VAL);

    ret = read_spi_reg(&icm42688_a, ICM42688_WHO_AM_I_REG, &val, false);
    validate_id("ICM42688P_A    (SPI2)", ret, val, ICM42688_WHO_AM_I_VAL);

    test_ms5611(&ms5611_a, "MS5611_A       (SPI2)");

    /* -----------------------------------------------------------------------
     * SPI3 — Blocco B
     * ----------------------------------------------------------------------- */
    const struct spi_dt_spec bmi088_accel_b =
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_accel_b), SPI_WORD_SET(8), 0);
    const struct spi_dt_spec bmi088_gyro_b =
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_gyro_b), SPI_WORD_SET(8), 0);
    const struct spi_dt_spec icm42688_b =
        SPI_DT_SPEC_GET(DT_NODELABEL(icm42688p_b), SPI_WORD_SET(8), 0);

    const struct device *spi3_bus = DEVICE_DT_GET(DT_NODELABEL(spi3));
    struct spi_dt_spec ms5611_b = {
        .bus = spi3_bus,
        .config = {
            .frequency = 1000000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 0,
            .cs = { .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi3), cs_gpios, 0), .delay = 0 },
        },
    };

    ret = read_spi_reg(&bmi088_accel_b, BMI088_ACCEL_CHIP_ID_REG, &val, true);
    validate_id("BMI088_Accel_B (SPI3)", ret, val, BMI088_ACCEL_CHIP_ID_VAL);

    ret = read_spi_reg(&bmi088_gyro_b, BMI088_GYRO_CHIP_ID_REG, &val, false);
    validate_id("BMI088_Gyro_B  (SPI3)", ret, val, BMI088_GYRO_CHIP_ID_VAL);

    ret = read_spi_reg(&icm42688_b, ICM42688_WHO_AM_I_REG, &val, false);
    validate_id("ICM42688P_B    (SPI3)", ret, val, ICM42688_WHO_AM_I_VAL);

    test_ms5611(&ms5611_b, "MS5611_B       (SPI3)");

    /* -----------------------------------------------------------------------
     * SPI4 — Ausiliari (tutti manual spec)
     * CS idx:  0=BMP585(disabled)  1=H3LIS  2=LIS2MDL  3=HSC
     * ----------------------------------------------------------------------- */
    const struct device *spi4_bus = DEVICE_DT_GET(DT_NODELABEL(spi4));

    struct spi_dt_spec h3lis331dl = {
        .bus = spi4_bus,
        .config = {
            .frequency = 1000000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 1,
            .cs = { .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi4), cs_gpios, 1), .delay = 0 },
        },
    };

    struct spi_dt_spec lis2mdl = {
        .bus = spi4_bus,
        .config = {
            .frequency = 1000000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 2,
            .cs = { .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi4), cs_gpios, 2), .delay = 0 },
        },
    };

    struct spi_dt_spec hsc_diff = {
        .bus = spi4_bus,
        .config = {
            .frequency = 800000,
            .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
            .slave = 3,
            .cs = { .gpio = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi4), cs_gpios, 3), .delay = 0 },
        },
    };

    ret = read_spi_reg(&h3lis331dl, H3LIS331DL_WHO_AM_I_REG, &val, false);
    validate_id("H3LIS331DL     (SPI4)", ret, val, H3LIS331DL_WHO_AM_I_VAL);

    ret = read_spi_reg(&lis2mdl, LIS2MDL_WHO_AM_I_REG, &val, false);
    validate_id("LIS2MDL        (SPI4)", ret, val, LIS2MDL_WHO_AM_I_VAL);

    /* HSC: sensore read-only. Output valido TF-B: 10%-90% di 2^14 = 1638..14746.
     * 0x0000 / 0xFFFF = MISO non guidato (CS non asserted o chip spento). */
    if (spi_is_ready_dt(&hsc_diff)) {
        uint8_t rx_hsc[2] = { 0 };
        struct spi_buf     rx_b   = { .buf = rx_hsc, .len = 2 };
        struct spi_buf_set rx_set = { .buffers = &rx_b, .count = 1 };

        ret = spi_read_dt(&hsc_diff, &rx_set);
        if (ret == 0) {
            uint8_t  status = (rx_hsc[0] >> 6) & 0x03;
            uint16_t raw    = ((uint16_t)(rx_hsc[0] & 0x3F) << 8) | rx_hsc[1];
            bool     valid  = (raw >= 1638U && raw <= 14746U);
            LOG_INF("[%s] HSC_DIFF (SPI4) -> raw=0x%04X status=%d",
                    valid ? "PASS" : "FAIL", raw, status);
        } else {
            LOG_ERR("[FAIL] HSC_DIFF (SPI4) -> Errore lettura (%d)", ret);
        }
    } else {
        LOG_ERR("[FAIL] HSC_DIFF (SPI4) -> Bus non pronto");
    }

    /* -----------------------------------------------------------------------
     * I2C4 — BNO055
     * ----------------------------------------------------------------------- */
    const struct i2c_dt_spec bno055 = I2C_DT_SPEC_GET(DT_NODELABEL(bno055));
    if (i2c_is_ready_dt(&bno055)) {
        ret = i2c_reg_read_byte_dt(&bno055, BNO055_CHIP_ID_REG, &val);
        validate_id("BNO055         (I2C4)", ret, val, BNO055_CHIP_ID_VAL);
    } else {
        LOG_ERR("[FAIL] BNO055 (I2C4) -> Bus non pronto");
    }

    LOG_INF("=== Test completato ===");

    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}