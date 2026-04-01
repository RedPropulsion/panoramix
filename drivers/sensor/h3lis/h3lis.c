#define DT_DRV_COMPAT st_h3lis331dl

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(h3lis331dl, CONFIG_SENSOR_LOG_LEVEL);

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

#include "h3lis.h"

/**
 * @brief Read a single register byte.
 *
 * @param dev   Sensor device pointer.
 * @param reg   Register address (H3LIS331DL_REG_*).
 * @param val   Pointer to store the read byte.
 * @return 0 on success, negative errno on failure.
 */
static inline int h3lis331dl_read_reg(const struct device *dev,
                                      uint8_t reg, uint8_t *val)
{
    return h3lis331dl_spi_read(dev, reg, val, 1);
}

/**
 * @brief Write a single register byte.
 *
 * @param dev   Sensor device pointer.
 * @param reg   Register address (H3LIS331DL_REG_*).
 * @param val   Byte value to write.
 * @return 0 on success, negative errno on failure.
 */
static inline int h3lis331dl_write_reg(const struct device *dev,
                                       uint8_t reg, uint8_t val)
{
    return h3lis331dl_spi_write(dev, reg, &val, 1);
}

/**
 * @brief Read-modify-write a register.
 *
 * Reads the current value, masks out the bits described by @p mask,
 * ORs in the new @p val bits, then writes the result back.
 *
 * @param dev   Sensor device pointer.
 * @param reg   Register address.
 * @param mask  Bitmask of bits to modify.
 * @param val   New bit values (must be pre-shifted to correct position).
 * @return 0 on success, negative errno on failure.
 */
static int h3lis331dl_update_reg(const struct device *dev,
                                 uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t current;
    int ret;

    ret = h3lis331dl_read_reg(dev, reg, &current);
    if (ret < 0) {
        return ret;
    }

    current = (current & ~mask) | (val & mask);
    return h3lis331dl_write_reg(dev, reg, current);
}


// Configuration helpers

/**
 * @brief Set power mode and output data rate via CTRL_REG1.
 *
 * Combines the power mode (PM[2:0]) and, for normal mode, the data
 * rate (DR[1:0]) into a single register write. Axis enables are
 * preserved via read-modify-write.
 *
 * @param dev    Sensor device pointer.
 * @param pm     Power mode bits  (H3LIS331DL_PM_*).
 * @param dr     Data rate bits   (H3LIS331DL_DR_*). Ignored in low-power mode.
 * @return 0 on success, negative errno on failure.
 */
static int h3lis331dl_set_odr_pm(const struct device *dev,
                                 uint8_t pm, uint8_t dr)
{
    return h3lis331dl_update_reg(dev, H3LIS331DL_REG_CTRL_REG1,
                                 H3LIS331DL_PM_MASK | H3LIS331DL_DR_MASK,
                                 pm | dr);
}

/**
 * @brief Set the measurement full-scale range via CTRL_REG4.
 *
 * Also updates the sensitivity stored in driver data.
 *
 * @param dev   Sensor device pointer.
 * @param fs    Full-scale bits (H3LIS331DL_FS_100G / 200G / 400G).
 * @return 0 on success, negative errno on failure.
 */
static int h3lis331dl_set_full_scale(const struct device *dev, uint8_t fs)
{
    struct h3lis331dl_data *data = dev->data;
    int ret;

    ret = h3lis331dl_update_reg(dev, H3LIS331DL_REG_CTRL_REG4,
                                H3LIS331DL_FS_MASK, fs);
    if (ret < 0) {
        return ret;
    }

    switch (fs) {
    case H3LIS331DL_FS_100G:
        data->sensitivity = H3LIS331DL_SENS_100G_MG_PER_DIGIT;
        break;
    case H3LIS331DL_FS_200G:
        data->sensitivity = H3LIS331DL_SENS_200G_MG_PER_DIGIT;
        break;
    case H3LIS331DL_FS_400G:
        data->sensitivity = H3LIS331DL_SENS_400G_MG_PER_DIGIT;
        break;
    default:
        LOG_ERR("Unknown full-scale value: 0x%02x", fs);
        return -EINVAL;
    }

    return 0;
}


/**
 * @brief Fetch a new acceleration sample from the sensor.
 *
 * Polls STATUS_REG until ZYXDA (all-axes data available) is set,
 * then reads all six output registers in a single burst transfer.
 *
 * Raw values are left-aligned 16-bit two's complement. The driver
 * right-shifts by 4 to obtain the 12-bit signed value.
 *
 * @param dev  Sensor device pointer.
 * @param chan Channel (SENSOR_CHAN_ALL accepted; individual axes not
 *             supported for burst fetch).
 * @return 0 on success, -ENODATA if data not ready, negative errno
 *         on bus errors.
 */
static int h3lis331dl_sample_fetch(const struct device *dev,
                                   enum sensor_channel chan)
{
    struct h3lis331dl_data *data = dev->data;
    uint8_t status;
    uint8_t raw[6];  /* OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H */
    int ret;

    if (chan != SENSOR_CHAN_ALL &&
        chan != SENSOR_CHAN_ACCEL_X &&
        chan != SENSOR_CHAN_ACCEL_Y &&
        chan != SENSOR_CHAN_ACCEL_Z &&
        chan != SENSOR_CHAN_ACCEL_XYZ) {
        return -ENOTSUP;
    }

    /* Wait for new data (ZYXDA bit in STATUS_REG) */
    ret = h3lis331dl_read_reg(dev, H3LIS331DL_REG_STATUS_REG, &status);
    if (ret < 0) {
        LOG_ERR("Failed to read STATUS_REG: %d", ret);
        return ret;
    }

    if (!(status & H3LIS331DL_ZYXDA)) {
        return -ENODATA;
    }

    /*
     * Burst-read 6 bytes starting at OUT_X_L (0x28).
     * The MS bit in the SPI command byte enables address
     * auto-increment so all six output registers are read in one
     * transaction (handled inside h3lis331dl_spi_read).
     */
    ret = h3lis331dl_spi_read(dev, H3LIS331DL_REG_OUT_X_L, raw, sizeof(raw));
    if (ret < 0) {
        LOG_ERR("Failed to read output registers: %d", ret);
        return ret;
    }

    /*
     * The device outputs data as two's complement, LSB first.
     * OUT_xL holds bits [7:0], OUT_xH holds bits [15:8].
     * Data is left-aligned: meaningful bits are [15:4] (12-bit).
     * Right-shift by 4 to get a signed 12-bit value in int16_t.
     */
    data->acc_x = (int16_t)sys_le16_to_cpu(raw[0] | (raw[1] << 8)) >> 4;
    data->acc_y = (int16_t)sys_le16_to_cpu(raw[2] | (raw[3] << 8)) >> 4;
    data->acc_z = (int16_t)sys_le16_to_cpu(raw[4] | (raw[5] << 8)) >> 4;

    LOG_DBG("raw: x=%d y=%d z=%d", data->acc_x, data->acc_y, data->acc_z);

    return 0;
}

/**
 * @brief Convert a raw 12-bit sample to a struct sensor_value.
 *
 * The result is expressed in m/s² (SI unit for acceleration).
 * Conversion: raw * sensitivity [mg/digit] * 9.80665e-3 [m/s² per mg]
 *
 * struct sensor_value uses integer + fractional micro parts:
 *   val1 = integer m/s²
 *   val2 = fractional µm/s²
 *
 * @param dev   Sensor device pointer.
 * @param chan  Sensor channel (ACCEL_X, ACCEL_Y, ACCEL_Z, or ACCEL_XYZ).
 * @param val   Output sensor_value pointer (array of 3 for XYZ).
 * @return 0 on success, -ENOTSUP for unsupported channels.
 */
static int h3lis331dl_channel_get(const struct device *dev,
                                  enum sensor_channel chan,
                                  struct sensor_value *val)
{
    const struct h3lis331dl_data *data = dev->data;
    int32_t raw;

    /* g = raw_12bit * sensitivity_mg_digit / 1000
     * m/s² = g * 9.80665
     * Combined: m/s² = raw * sensitivity * 9.80665 / 1000
     * We use integer arithmetic scaled to preserve precision.
     * 9.80665 * sensitivity / 1000 expressed as:
     *   integer part  = (raw * sensitivity * 9) / 1000  + carry
     *   fractional µ  = remaining * 80665 / 100
     */

    auto_convert:
    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        raw = data->acc_x;
        break;
    case SENSOR_CHAN_ACCEL_Y:
        raw = data->acc_y;
        break;
    case SENSOR_CHAN_ACCEL_Z:
        raw = data->acc_z;
        break;
    case SENSOR_CHAN_ACCEL_XYZ:
        /* Recurse for each axis */
        h3lis331dl_channel_get(dev, SENSOR_CHAN_ACCEL_X, &val[0]);
        h3lis331dl_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &val[1]);
        h3lis331dl_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &val[2]);
        return 0;
    default:
        return -ENOTSUP;
    }

    /*
     * Acceleration in µm/s²:
     *   a_um_s2 = raw * sensitivity_mg * 9806.65  (9.80665 m/s² -> 9806650 µm/s²)
     *           / 1000                             (mg -> g)
     * To avoid 32-bit overflow for ±400 g range:
     */
    int64_t a_um_s2 = (int64_t)raw * data->sensitivity * 9807LL;

    val->val1 = (int32_t)(a_um_s2 / 1000000LL);
    val->val2 = (int32_t)(a_um_s2 % 1000000LL);

    /* Ensure val2 has the same sign as val1 */
    if (val->val1 > 0 && val->val2 < 0) {
        val->val1--;
        val->val2 += 1000000;
    } else if (val->val1 < 0 && val->val2 > 0) {
        val->val1++;
        val->val2 -= 1000000;
    }

    return 0;

}


/**
 * @brief Set a sensor attribute.
 *
 * Supported attributes:
 *   SENSOR_ATTR_SAMPLING_FREQUENCY - maps to ODR setting
 *   SENSOR_ATTR_FULL_SCALE         - maps to full-scale ±g range
 *
 * @param dev   Sensor device.
 * @param chan  Must be SENSOR_CHAN_ACCEL_XYZ (global setting).
 * @param attr  Attribute to set.
 * @param val   New value.
 * @return 0 on success, -ENOTSUP/-EINVAL on invalid input.
 */
static int h3lis331dl_attr_set(const struct device *dev,
                               enum sensor_channel chan,
                               enum sensor_attribute attr,
                               const struct sensor_value *val)
{
    if (chan != SENSOR_CHAN_ACCEL_XYZ) {
        return -ENOTSUP;
    }

    switch (attr) {
    case SENSOR_ATTR_SAMPLING_FREQUENCY: {
        /* val->val1 is the desired ODR in Hz */
        uint8_t pm, dr;

        if (val->val1 <= 0) {
            /* Power down */
            pm = H3LIS331DL_PM_POWER_DOWN;
            dr = H3LIS331DL_DR_50HZ;
        } else if (val->val1 <= 10) {
            /* Low-power mode - select closest ODR */
            pm = (val->val1 <= 1)  ? H3LIS331DL_PM_LOW_POWER_1HZ  :
                 (val->val1 <= 2)  ? H3LIS331DL_PM_LOW_POWER_2HZ  :
                 (val->val1 <= 5)  ? H3LIS331DL_PM_LOW_POWER_5HZ  :
                                     H3LIS331DL_PM_LOW_POWER_10HZ;
            dr = H3LIS331DL_DR_50HZ; /* DR irrelevant in LP mode */
        } else {
            /* Normal mode */
            pm = H3LIS331DL_PM_NORMAL;
            dr = (val->val1 <= 50)  ? H3LIS331DL_DR_50HZ   :
                 (val->val1 <= 100) ? H3LIS331DL_DR_100HZ  :
                 (val->val1 <= 400) ? H3LIS331DL_DR_400HZ  :
                                      H3LIS331DL_DR_1000HZ;
        }
        return h3lis331dl_set_odr_pm(dev, pm, dr);
    }

    case SENSOR_ATTR_FULL_SCALE: {
        /* val->val1 is the desired full scale in g */
        uint8_t fs;

        if (val->val1 <= 100) {
            fs = H3LIS331DL_FS_100G;
        } else if (val->val1 <= 200) {
            fs = H3LIS331DL_FS_200G;
        } else {
            fs = H3LIS331DL_FS_400G;
        }
        return h3lis331dl_set_full_scale(dev, fs);
    }

    default:
        return -ENOTSUP;
    }
}


/**
 * @brief Initialise the H3LIS331DL sensor.
 *
 * Steps:
 *  1. Initialise the underlying bus .
 *  2. Verify WHO_AM_I .
 *  3. Issue a soft BOOT to restore factory calibration.
 *  4. Enable BDU to avoid reading mixed-sample data.
 *  5. Configure ODR and full-scale from DT config.
 *  6. Enable all three axes in normal mode.
 *
 * @param dev  Sensor device pointer.
 * @return 0 on success, negative errno on failure.
 */
static int h3lis331dl_init(const struct device *dev)
{
    printk("h3lis331dl_init called\n");
    const struct h3lis331dl_dev_config *cfg = dev->config;
    struct h3lis331dl_data *data = dev->data;
    uint8_t chip_id;
    int ret;

    // init bus
    ret = h3lis331dl_spi_init(dev);
    if (ret < 0) {
        LOG_ERR("Bus init failed: %d", ret);
        return -ret; 
    }

    // WHO AM I ? 
    ret = h3lis331dl_read_reg(dev, H3LIS331DL_REG_WHO_AM_I, &chip_id);
    if (ret < 0) {
        LOG_ERR("WHO_AM_I read failed: %d", ret);
        return -ret;
    }
    if (chip_id != H3LIS331DL_DEVICE_ID) { //chip id is zero
        LOG_ERR("Wrong device ID: expected 0x%02x, got 0x%02x",
                H3LIS331DL_DEVICE_ID, chip_id);
        return (int)chip_id + 5; // ENODEV
    }
    LOG_INF("H3LIS331DL found, WHO_AM_I=0x%02x", chip_id);

    // soft reboot to restore NVM calibration 
    ret = h3lis331dl_write_reg(dev, H3LIS331DL_REG_CTRL_REG2,
                               H3LIS331DL_BOOT);
    if (ret < 0) {
        LOG_ERR("BOOT write failed: %d", ret);
        return -ret;
    }
    /*
     * The BOOT bit self-clears when reboot is complete.
     * Poll until cleared (typically < 5 ms).
     */
    uint8_t ctrl2;
    int attempts = 50;

    do {
        k_msleep(1);
        ret = h3lis331dl_read_reg(dev, H3LIS331DL_REG_CTRL_REG2, &ctrl2);
        if (ret < 0) {
            return -ret;
        }
    } while ((ctrl2 & H3LIS331DL_BOOT) && (--attempts > 0));

    if (ctrl2 & H3LIS331DL_BOOT) {
        LOG_ERR("BOOT timeout - device did not respond");
        return -ret; //ETIMEDOUT
    }

    // enable Block Data Update 
    ret = h3lis331dl_write_reg(dev, H3LIS331DL_REG_CTRL_REG4,
                               H3LIS331DL_BDU);
    if (ret < 0) {
        LOG_ERR("CTRL_REG4 write failed: %d", ret);
        return -ret;
    }

    // Set full-scale range from DT config 
    ret = h3lis331dl_set_full_scale(dev, cfg->full_scale);
    if (ret < 0) {
        LOG_ERR("Full-scale config failed: %d", ret);
        return ret;
    }

    // normal mode, configured ODR, all axes enabled
    uint8_t ctrl1 = H3LIS331DL_PM_NORMAL |
                    cfg->odr             |
                    H3LIS331DL_ALL_AXES_EN;

    ret = h3lis331dl_write_reg(dev, H3LIS331DL_REG_CTRL_REG1, ctrl1);
    if (ret < 0) {
        LOG_ERR("CTRL_REG1 write failed: %d", ret);
        return ret;
    }

    LOG_INF("H3LIS331DL initialised: ODR=0x%02x FS=0x%02x sens=%d mg/digit",
            cfg->odr, cfg->full_scale, data->sensitivity);

    return 0;
}


static const struct sensor_driver_api h3lis331dl_api = {
    .sample_fetch = h3lis331dl_sample_fetch,
    .channel_get  = h3lis331dl_channel_get,
    .attr_set     = h3lis331dl_attr_set,
};

// Device Tree Macro 
#define H3LIS331DL_DEFINE(inst)                                              \
    static struct h3lis331dl_data h3lis331dl_data_##inst;                    \
                                                                              \
    static const struct h3lis331dl_dev_config h3lis331dl_cfg_##inst = {      \
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |SPI_TRANSFER_MSB, 0), \
        .full_scale = DT_INST_PROP_OR(inst, full_scale, H3LIS331DL_FS_100G), \
        .odr        = DT_INST_PROP_OR(inst, odr,        H3LIS331DL_DR_100HZ),\
    };                                                                        \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                                        \
                                 h3lis331dl_init,                              \
                                 NULL,                                         \
                                 &h3lis331dl_data_##inst,                      \
                                 &h3lis331dl_cfg_##inst,                       \
                                 POST_KERNEL,                                  \
                                 CONFIG_SENSOR_INIT_PRIORITY,                  \
                                 &h3lis331dl_api);

DT_INST_FOREACH_STATUS_OKAY(H3LIS331DL_DEFINE)