#ifndef ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_H_
#define ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif


struct h3lis331dl_dev_config {
    struct spi_dt_spec spi;
    uint8_t full_scale;  
    uint8_t odr;       
};

// Driver runtime data
struct h3lis331dl_data {
    /* Raw 12-bit two's complement samples (left-aligned in 16-bit) */
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    /* Sensitivity in mg/digit, derived from full-scale setting */
    uint16_t sensitivity;

#ifdef CONFIG_H3LIS331DL_TRIGGER
    const struct device *dev;
    struct gpio_callback gpio_cb;
    sensor_trigger_handler_t drdy_handler;
    struct sensor_trigger drdy_trigger;
    struct k_work work;
#endif
};


int h3lis331dl_spi_init(const struct device *dev);
int h3lis331dl_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, uint8_t len);
int h3lis331dl_spi_write(const struct device *dev, uint8_t reg, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_H3LIS331DL_H_ */