#define DT_DRV_COMPAT st_h3lis331dl

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(h3lis331dl, CONFIG_SENSOR_LOG_LEVEL);

#include <zephyr/drivers/spi.h>
#include "h3lis.h"



/**
 * @brief Read one or more consecutive registers over SPI.
 *
 * Constructs the 8-bit command byte:
 *   bit 7 = 1  (read direction)
 *   bit 6 = 1  if len > 1 (auto-increment address)
 *   bits [5:0] = register address
 *
 * The transfer uses two SPI buffers:
 *   tx[0]: command byte  (1 byte)
 *   rx[0]: dummy byte    (1 byte, discarded - received during command phase)
 *   rx[1]: actual data   (len bytes)
 *
 * @param dev   Sensor device pointer.
 * @param reg   Starting register address.
 * @param data  Buffer to receive the data.
 * @param len   Number of bytes to read.
 * @return 0 on success, negative errno on SPI error.
 */
int h3lis331dl_spi_read(const struct device *dev,
                        uint8_t reg, uint8_t *data, uint8_t len)
{
    const struct h3lis331dl_dev_config *cfg = dev->config;

    uint8_t cmd = H3LIS331DL_SPI_READ | (reg & 0x3F);

    if (len > 1) {
        cmd |= H3LIS331DL_SPI_AUTO_INC;
    }

    const struct spi_buf tx_buf = {
        .buf = &cmd,
        .len = 1,
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count   = 1,
    };

    uint8_t dummy;
    struct spi_buf rx_bufs[2] = {
        { .buf = &dummy, .len = 1   },
        { .buf = data,   .len = len },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count   = 2,
    };

    return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

/**
 * @brief Write one or more consecutive registers over SPI.
 *
 * Constructs the 8-bit command byte:
 *   bit 7 = 0  (write direction)
 *   bit 6 = 1  if len > 1 (auto-increment address)
 *   bits [5:0] = register address
 *
 * TX buffer layout: [cmd][data[0]][data[1]]...[data[len-1]]
 * A temporary local buffer is used to prepend the command byte.
 *
 * @param dev   Sensor device pointer.
 * @param reg   Starting register address.
 * @param data  Bytes to write.
 * @param len   Number of bytes to write.
 * @return 0 on success, negative errno on SPI error or buffer overflow.
 */
int h3lis331dl_spi_write(const struct device *dev,
                         uint8_t reg, uint8_t *data, uint8_t len)
{
    const struct h3lis331dl_dev_config *cfg = dev->config;

    uint8_t cmd = H3LIS331DL_SPI_WRITE | (reg & 0x3F);

    if (len > 1) {
        cmd |= H3LIS331DL_SPI_AUTO_INC;
    }

    struct spi_buf tx_bufs[2] = {
        { .buf = &cmd, .len = 1   },
        { .buf = data, .len = len },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count   = 2,
    };

    return spi_write_dt(&cfg->spi, &tx);
}

/* ============================================================
 * SPI bus initialisation
 * ============================================================ */

/**
 * @brief Verify the SPI bus is ready.
 *
 * @param dev  Sensor device pointer.
 * @return 0 if bus is ready, -ENODEV otherwise.
 */
int h3lis331dl_spi_init(const struct device *dev)
{
    const struct h3lis331dl_dev_config *cfg = dev->config;

    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("SPI bus not ready: %s", cfg->spi.bus->name);
        return -15;
    }

    LOG_DBG("SPI bus ready: %s", cfg->spi.bus->name);

    return 0;
}