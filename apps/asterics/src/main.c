/*
 * AsterICS – Validazione Hardware Completa dei Sensori
 *
 * Struttura dei test per bus SPI/I2C:
 *   SPI5  → Blocco MCU:            MS5611, BMI088 Gyro/Accel, ICM42688P, [BMP585*]
 *   SPI2  → Blocco Primario (A):   MS5611, BMI088 Gyro/Accel, ICM42688P
 *   SPI3  → Blocco Secondario (B): MS5611, BMI088 Gyro/Accel, ICM42688P
 *   SPI4  → Sensori Ausiliari:     [BMP585*], H3LIS331DL, LIS2MDL, HSC Diff
 *   I2C4  → Orientamento/Corrente: BNO055, INA219 x6
 *   I2C2  → Termocoppia:           MCP9600 x2
 *
 * (*) I nodi mcu_bmp585 e bmp585_sens hanno status = "disabled" nel DTS
 *     perché il driver Zephyr non supporta ancora la modalità SPI.
 *     L'accesso raw SPI funziona: imposta status = "okay" nel DTS per
 *     abilitare il test (CHIP_ID a reg 0x01, valore atteso 0x50).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_hw_validation, LOG_LEVEL_INF);

/* Flag SPI: Master, 8-bit, MSB-first, CPOL=0, CPHA=0 */
#define SPI_OP_FLAGS  (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER)

/* ─────────────────────────────────────────────────────────────
 *  Costanti registri SPI MEMS
 *  Protocollo standard: primo byte = indirizzo | 0x80 (read bit),
 *  secondo byte = dummy (0x00), terzo byte = dato letto.
 *  BMI088 Accel: richiede un dummy byte aggiuntivo (3 byte totali).
 * ───────────────────────────────────────────────────────────── */
#define BMI088_ACCEL_CHIP_ID_REG   0x00
#define BMI088_ACCEL_CHIP_ID_VAL   0x1E
#define BMI088_GYRO_CHIP_ID_REG    0x00
#define BMI088_GYRO_CHIP_ID_VAL    0x0F
#define ICM42688_WHO_AM_I_REG      0x75
#define ICM42688_WHO_AM_I_VAL      0x47
#define H3LIS331DL_WHO_AM_I_REG    0x0F
#define H3LIS331DL_WHO_AM_I_VAL    0x32
#define LIS2MDL_WHO_AM_I_REG       0x4F
#define LIS2MDL_WHO_AM_I_VAL       0x40
#define BMP585_CHIP_ID_REG         0x01   /* BMP581/585: CHIP_ID @ 0x01 */
#define BMP585_CHIP_ID_VAL         0x51

/* ─────────────────────────────────────────────────────────────
 *  Costanti registri I2C
 * ───────────────────────────────────────────────────────────── */
#define BNO055_CHIP_ID_REG         0x00
#define BNO055_CHIP_ID_VAL         0xA0
/* MCP9600: Device ID a reg 0x20. Il master legge 1 byte (MSB del word a 16-bit).
 * Device ID upper nibble = 0x4x → MSB atteso = 0x40. */
#define MCP9600_DEV_ID_REG         0x20
#define MCP9600_DEV_ID_MSB_VAL     0x40
/* INA219: CONFIG register @ 0x00. Power-on reset = 0x399F → MSB = 0x39.
 * i2c_reg_read_byte legge solo il MSB della risposta a 16-bit. */
#define INA219_CONFIG_REG          0x00
#define INA219_CONFIG_MSB_VAL      0x39

/* ═════════════════════════════════════════════════════════════
 *  Struttura per test MEMS standard (WHO_AM_I / CHIP_ID via SPI)
 * ═════════════════════════════════════════════════════════════ */
struct spi_sensor_test {
    const char        *name;
    struct spi_dt_spec spi;
    uint8_t            id_reg;
    uint8_t            expected_id;
    bool               needs_dummy_byte;
};

/* ─── SPI5: Blocco MCU ─── */
static const struct spi_sensor_test spi5_sensors[] = {
    {
        "BMI088 Gyro (MCU)",
        SPI_DT_SPEC_GET(DT_NODELABEL(mcu_bmi088_gyro),  SPI_OP_FLAGS),
        BMI088_GYRO_CHIP_ID_REG,  BMI088_GYRO_CHIP_ID_VAL,  false
    },
    {
        "BMI088 Accel (MCU)",
        SPI_DT_SPEC_GET(DT_NODELABEL(mcu_bmi088_accel), SPI_OP_FLAGS),
        BMI088_ACCEL_CHIP_ID_REG, BMI088_ACCEL_CHIP_ID_VAL, true
    },
    {
        "ICM42688P (MCU)",
        SPI_DT_SPEC_GET(DT_NODELABEL(mcu_icm42688p),    SPI_OP_FLAGS),
        ICM42688_WHO_AM_I_REG,    ICM42688_WHO_AM_I_VAL,    false
    },
};
/* MS5611: protocollo diverso dai MEMS, gestito da validate_ms5611() */
static const struct spi_dt_spec spi5_ms5611 =
    SPI_DT_SPEC_GET(DT_NODELABEL(mcu_ms5611), SPI_OP_FLAGS);

/* BMP585 MCU: abilitare con status = "okay" nel DTS */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcu_bmp585), okay)
static const struct spi_sensor_test spi5_bmp585 = {
    "BMP585 (MCU)",
    SPI_DT_SPEC_GET(DT_NODELABEL(mcu_bmp585), SPI_OP_FLAGS),
    BMP585_CHIP_ID_REG, BMP585_CHIP_ID_VAL, false
};
#endif

/* ─── SPI2: Blocco Primario (A) ─── */
static const struct spi_sensor_test spi2_sensors[] = {
    {
        "BMI088 Gyro (A)",
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_gyro_a),  SPI_OP_FLAGS),
        BMI088_GYRO_CHIP_ID_REG,  BMI088_GYRO_CHIP_ID_VAL,  false
    },
    {
        "BMI088 Accel (A)",
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_accel_a), SPI_OP_FLAGS),
        BMI088_ACCEL_CHIP_ID_REG, BMI088_ACCEL_CHIP_ID_VAL, true
    },
    {
        "ICM42688P (A)",
        SPI_DT_SPEC_GET(DT_NODELABEL(icm42688p_a),    SPI_OP_FLAGS),
        ICM42688_WHO_AM_I_REG,    ICM42688_WHO_AM_I_VAL,    false
    },
};
static const struct spi_dt_spec spi2_ms5611 =
    SPI_DT_SPEC_GET(DT_NODELABEL(ms5611_a), SPI_OP_FLAGS);

/* ─── SPI3: Blocco Secondario (B) ─── */
static const struct spi_sensor_test spi3_sensors[] = {
    {
        "BMI088 Gyro (B)",
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_gyro_b),  SPI_OP_FLAGS),
        BMI088_GYRO_CHIP_ID_REG,  BMI088_GYRO_CHIP_ID_VAL,  false
    },
    {
        "BMI088 Accel (B)",
        SPI_DT_SPEC_GET(DT_NODELABEL(bmi088_accel_b), SPI_OP_FLAGS),
        BMI088_ACCEL_CHIP_ID_REG, BMI088_ACCEL_CHIP_ID_VAL, true
    },
    {
        "ICM42688P (B)",
        SPI_DT_SPEC_GET(DT_NODELABEL(icm42688p_b),    SPI_OP_FLAGS),
        ICM42688_WHO_AM_I_REG,    ICM42688_WHO_AM_I_VAL,    false
    },
};
static const struct spi_dt_spec spi3_ms5611 =
    SPI_DT_SPEC_GET(DT_NODELABEL(ms5611_b), SPI_OP_FLAGS);

/* ─── SPI4: Sensori Ausiliari ─── */
static const struct spi_sensor_test spi4_sensors[] = {
    {
        "H3LIS331DL",
        SPI_DT_SPEC_GET(DT_NODELABEL(h3lis331dl_sens), SPI_OP_FLAGS),
        H3LIS331DL_WHO_AM_I_REG, H3LIS331DL_WHO_AM_I_VAL, false
    },
    {
        "LIS2MDL",
        SPI_DT_SPEC_GET(DT_NODELABEL(lis2mdl_sens),    SPI_OP_FLAGS),
        LIS2MDL_WHO_AM_I_REG,    LIS2MDL_WHO_AM_I_VAL,    false
    },
};
/* HSC: nessun registro, protocollo a frame puro → validate_hsc() */
static const struct spi_dt_spec spi4_hsc =
    SPI_DT_SPEC_GET(DT_NODELABEL(hsc_diff), SPI_OP_FLAGS);

/* BMP585 Sens: abilitare con status = "okay" nel DTS */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bmp585_sens), okay)
static const struct spi_sensor_test spi4_bmp585 = {
    "BMP585 (Sens)",
    SPI_DT_SPEC_GET(DT_NODELABEL(bmp585_sens), SPI_OP_FLAGS),
    BMP585_CHIP_ID_REG, BMP585_CHIP_ID_VAL, false
};
#endif

/* ═════════════════════════════════════════════════════════════
 *  Struttura per test I2C generico (lettura registro singolo)
 * ═════════════════════════════════════════════════════════════ */
struct i2c_sensor_test {
    const char        *name;
    struct i2c_dt_spec i2c;
    uint8_t            id_reg;
    uint8_t            expected_id;
};

/* ─── I2C4: BNO055 + INA219 x6 ─── */
static const struct i2c_sensor_test i2c4_sensors[] = {
    { "BNO055",        I2C_DT_SPEC_GET(DT_NODELABEL(bno055)),        BNO055_CHIP_ID_REG, BNO055_CHIP_ID_VAL    },
    { "INA219 MCU",    I2C_DT_SPEC_GET(DT_NODELABEL(ina219_mcu)),    INA219_CONFIG_REG,  INA219_CONFIG_MSB_VAL },
    { "INA219 RF",     I2C_DT_SPEC_GET(DT_NODELABEL(ina219_rf)),     INA219_CONFIG_REG,  INA219_CONFIG_MSB_VAL },
    { "INA219 Servo",  I2C_DT_SPEC_GET(DT_NODELABEL(ina219_servo)),  INA219_CONFIG_REG,  INA219_CONFIG_MSB_VAL },
    { "INA219 Ring",   I2C_DT_SPEC_GET(DT_NODELABEL(ina219_ring)),   INA219_CONFIG_REG,  INA219_CONFIG_MSB_VAL },
    { "INA219 Sensor", I2C_DT_SPEC_GET(DT_NODELABEL(ina219_sensor)), INA219_CONFIG_REG,  INA219_CONFIG_MSB_VAL },
    { "INA219 Camera", I2C_DT_SPEC_GET(DT_NODELABEL(ina219_camera)), INA219_CONFIG_REG,  INA219_CONFIG_MSB_VAL },
};

/* ─── I2C2: MCP9600 x2 ─── */
static const struct i2c_sensor_test i2c2_sensors[] = {
    { "MCP9600 (0x40)", I2C_DT_SPEC_GET(DT_NODELABEL(mcp9600)),   MCP9600_DEV_ID_REG, MCP9600_DEV_ID_MSB_VAL },
    { "MCP9600 (0x4F)", I2C_DT_SPEC_GET(DT_NODELABEL(mcp9600_2)), MCP9600_DEV_ID_REG, MCP9600_DEV_ID_MSB_VAL },
};

/* ═════════════════════════════════════════════════════════════
 *  Contatori globali pass/fail
 * ═════════════════════════════════════════════════════════════ */
static int g_pass;
static int g_fail;

/* ═════════════════════════════════════════════════════════════
 *  FUNZIONE 1: MEMS standard con WHO_AM_I / CHIP_ID
 *
 *  Protocollo: [reg | 0x80] [dummy?] → [don't-care] [id]
 *  Buffer allineati a 32 byte per sicurezza D-Cache Cortex-M7.
 * ═════════════════════════════════════════════════════════════ */
static int validate_spi_sensor(const struct spi_sensor_test *s)
{
    if (!spi_is_ready_dt(&s->spi)) {
        LOG_ERR("[%s] SPI Bus o GPIO CS non pronti!", s->name);
        g_fail++;
        return -ENODEV;
    }

    __aligned(32) uint8_t tx[3] = { s->id_reg | 0x80, 0x00, 0x00 };
    __aligned(32) uint8_t rx[3] = { 0 };
    size_t len = s->needs_dummy_byte ? 3 : 2;

    struct spi_buf     tx_b   = { .buf = tx, .len = len };
    struct spi_buf     rx_b   = { .buf = rx, .len = len };
    struct spi_buf_set tx_set = { .buffers = &tx_b, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_b, .count = 1 };

    int ret = spi_transceive_dt(&s->spi, &tx_set, &rx_set);
    if (ret != 0) {
        LOG_ERR("[%s] Errore transazione HW: %d", s->name, ret);
        g_fail++;
        return ret;
    }

    uint8_t val = s->needs_dummy_byte ? rx[2] : rx[1];

    if (val == s->expected_id) {
        LOG_INF("[PASS] %-22s ID: 0x%02X", s->name, val);
        g_pass++;
        return 0;
    }

    LOG_WRN("[FAIL] %-22s Letto: 0x%02X, Atteso: 0x%02X",
            s->name, val, s->expected_id);
    g_fail++;
    return -EIO;
}

/* ═════════════════════════════════════════════════════════════
 *  FUNZIONE 2: MS5611 – Reset + lettura PROM[C1]
 *
 *  Il MS5611 non ha un registro WHO_AM_I. La presenza del sensore
 *  viene verificata inviando un RESET (0x1E), aspettando il reload
 *  (>2.8 ms), e poi leggendo il coefficiente di calibrazione C1
 *  (cmd 0xA2). Un valore valido è qualsiasi word != 0x0000/0xFFFF.
 * ═════════════════════════════════════════════════════════════ */
static int validate_ms5611(const char *name, const struct spi_dt_spec *spi)
{
    if (!spi_is_ready_dt(spi)) {
        LOG_ERR("[%s] SPI non pronto!", name);
        g_fail++;
        return -ENODEV;
    }

    /* Step 1: RESET */
    __aligned(32) uint8_t rst_cmd = 0x1E;
    struct spi_buf     tx_rst = { .buf = &rst_cmd, .len = 1 };
    struct spi_buf_set tx_set = { .buffers = &tx_rst, .count = 1 };

    int ret = spi_write_dt(spi, &tx_set);
    if (ret != 0) {
        LOG_ERR("[%s] Errore RESET: %d", name, ret);
        g_fail++;
        return ret;
    }

    k_sleep(K_MSEC(3)); /* Datasheet t_reload > 2.8 ms */

    /* Step 2: Leggi PROM[C1] con comando 0xA2 (3 byte totali: cmd + 2 dati) */
    __aligned(32) uint8_t tx_p[3] = { 0xA2, 0x00, 0x00 };
    __aligned(32) uint8_t rx_p[3] = { 0 };
    struct spi_buf     tx_pb  = { .buf = tx_p, .len = 3 };
    struct spi_buf     rx_pb  = { .buf = rx_p, .len = 3 };
    struct spi_buf_set tx_ps  = { .buffers = &tx_pb, .count = 1 };
    struct spi_buf_set rx_ps  = { .buffers = &rx_pb, .count = 1 };

    ret = spi_transceive_dt(spi, &tx_ps, &rx_ps);
    if (ret != 0) {
        LOG_ERR("[%s] Errore lettura PROM: %d", name, ret);
        g_fail++;
        return ret;
    }

    uint16_t prom_c1 = ((uint16_t)rx_p[1] << 8) | rx_p[2];

    if (prom_c1 != 0x0000 && prom_c1 != 0xFFFF) {
        LOG_INF("[PASS] %-22s PROM[C1]: 0x%04X", name, prom_c1);
        g_pass++;
        return 0;
    }

    LOG_WRN("[FAIL] %-22s PROM[C1] non valido: 0x%04X", name, prom_c1);
    g_fail++;
    return -EIO;
}

/* ═════════════════════════════════════════════════════════════
 *  FUNZIONE 3: HSC Honeywell – lettura frame a 4 byte
 *
 *  Il sensore HSC non usa un bus a registro. Alla sola asserzione
 *  del CS clocka fuori 4 byte: [status(2b)|pressH(14b)] [pressL]
 *  [tempH(11b)] [tempL]. Bit [7:6] byte 0 = status:
 *    00 = Normal  01 = Reserved  10 = Stale data  11 = Fault
 *  Il test passa se status != 0b11 (fault).
 * ═════════════════════════════════════════════════════════════ */
static int validate_hsc(const char *name, const struct spi_dt_spec *spi)
{
    if (!spi_is_ready_dt(spi)) {
        LOG_ERR("[%s] SPI non pronto!", name);
        g_fail++;
        return -ENODEV;
    }

    __aligned(32) uint8_t rx[4] = { 0 };
    struct spi_buf     rx_b   = { .buf = rx, .len = 4 };
    struct spi_buf_set rx_set = { .buffers = &rx_b, .count = 1 };

    int ret = spi_read_dt(spi, &rx_set);
    if (ret != 0) {
        LOG_ERR("[%s] Errore lettura frame: %d", name, ret);
        g_fail++;
        return ret;
    }

    uint8_t status = (rx[0] >> 6) & 0x03;

    if (status != 0x03) {
        LOG_INF("[PASS] %-22s Status: 0x%X  raw: %02X %02X %02X %02X",
                name, status, rx[0], rx[1], rx[2], rx[3]);
        g_pass++;
        return 0;
    }

    LOG_WRN("[FAIL] %-22s FAULT (status bits = 11)", name);
    g_fail++;
    return -EIO;
}

/* ═════════════════════════════════════════════════════════════
 *  FUNZIONE 4: Sensore I2C generico – lettura byte singolo
 *
 *  Usata per BNO055 (CHIP_ID), INA219 (CONFIG MSB),
 *  MCP9600 (Device ID MSB). i2c_reg_read_byte_dt legge il primo
 *  byte della risposta del registro puntato.
 * ═════════════════════════════════════════════════════════════ */
static int validate_i2c_sensor(const struct i2c_sensor_test *s)
{
    if (!i2c_is_ready_dt(&s->i2c)) {
        LOG_ERR("[%s] Bus I2C non pronto!", s->name);
        g_fail++;
        return -ENODEV;
    }

    uint8_t val = 0;
    int ret = i2c_reg_read_byte_dt(&s->i2c, s->id_reg, &val);

    if (ret != 0) {
        LOG_WRN("[FAIL] %-22s Errore I2C (ret: %d)", s->name, ret);
        g_fail++;
        return ret;
    }

    if (val == s->expected_id) {
        LOG_INF("[PASS] %-22s Reg[0x%02X]: 0x%02X", s->name, s->id_reg, val);
        g_pass++;
        return 0;
    }

    LOG_WRN("[FAIL] %-22s Letto: 0x%02X, Atteso: 0x%02X",
            s->name, val, s->expected_id);
    g_fail++;
    return -EIO;
}

/* ─────────────────────────────────────────────────────────────
 *  Helper: itera un array spi_sensor_test
 * ───────────────────────────────────────────────────────────── */
static void run_spi_array(const struct spi_sensor_test *arr, int n)
{
    for (int i = 0; i < n; i++) {
        validate_spi_sensor(&arr[i]);
    }
}

/* ─────────────────────────────────────────────────────────────
 *  Helper: itera un array i2c_sensor_test
 * ───────────────────────────────────────────────────────────── */
static void run_i2c_array(const struct i2c_sensor_test *arr, int n)
{
    for (int i = 0; i < n; i++) {
        validate_i2c_sensor(&arr[i]);
    }
}

/* ═════════════════════════════════════════════════════════════
 *  MAIN
 * ═════════════════════════════════════════════════════════════ */
int main(void)
{
    LOG_INF("=========================================");
    LOG_INF("  AsterICS - Validazione Hardware Sensori");
    LOG_INF("=========================================");

    /* ── SPI5: Blocco MCU ─────────────────────────────────── */
    LOG_INF("----- SPI5: Blocco MCU ------------------");
    validate_ms5611("MS5611 (MCU)", &spi5_ms5611);
    run_spi_array(spi5_sensors, ARRAY_SIZE(spi5_sensors));
    k_sleep(K_MSEC(100));

#if DT_NODE_HAS_STATUS(DT_NODELABEL(mcu_bmp585), okay)
    validate_spi_sensor(&spi5_bmp585);
#else
    LOG_WRN("[SKIP] BMP585 (MCU)     -> status=disabled nel DTS");
#endif

    /* ── SPI2: Blocco Primario (A) ───────────────────────── */
    LOG_INF("----- SPI2: Blocco Primario (A) ---------");
    validate_ms5611("MS5611 (A)", &spi2_ms5611);
    run_spi_array(spi2_sensors, ARRAY_SIZE(spi2_sensors));

    /* ── SPI3: Blocco Secondario (B) ─────────────────────── */
    LOG_INF("----- SPI3: Blocco Secondario (B) -------");
    validate_ms5611("MS5611 (B)", &spi3_ms5611);
    run_spi_array(spi3_sensors, ARRAY_SIZE(spi3_sensors));

    /* ── SPI4: Sensori Ausiliari ──────────────────────────── */
    LOG_INF("----- SPI4: Sensori Ausiliari -----------");
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bmp585_sens), okay)
    validate_spi_sensor(&spi4_bmp585);
#else
    LOG_WRN("[SKIP] BMP585 (Sens)    -> status=disabled nel DTS");
#endif
    run_spi_array(spi4_sensors, ARRAY_SIZE(spi4_sensors));
    validate_hsc("HSC Diff", &spi4_hsc);

    /* ── I2C4: BNO055 + INA219 x6 ────────────────────────── */
    LOG_INF("----- I2C4: Orientamento / Corrente -----");
    run_i2c_array(i2c4_sensors, ARRAY_SIZE(i2c4_sensors));

    /* ── I2C2: MCP9600 Termocoppia x2 ────────────────────── */
    LOG_INF("----- I2C2: Termocoppia -----------------");
    run_i2c_array(i2c2_sensors, ARRAY_SIZE(i2c2_sensors));

    /* ── Riepilogo ────────────────────────────────────────── */
    LOG_INF("=========================================");
    LOG_INF("  Risultato: %d PASS  %d FAIL  (tot: %d)",
            g_pass, g_fail, g_pass + g_fail);
    LOG_INF("=========================================");

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}