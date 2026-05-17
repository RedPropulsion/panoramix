#ifndef SENSOR_INA219_ASYNC_BUS_H_
#define SENSOR_INA219_ASYNC_BUS_H_

#include <zephyr/device.h>
#include <zephyr/rtio/sqe.h>

#include <stdint.h>

struct ina219_bus {
  struct {
    struct rtio *ctx;
    struct rtio_iodev *iodev;
  } rtio;
};

int ina219_prep_reg_read_rtio_async(const struct ina219_bus *bus,
                                    uint8_t reg_addr, uint8_t *raw_reg_data,
                                    struct rtio_sqe **out);

int ina219_prep_reg_write_rtio_async(const struct ina219_bus *bus,
                                     uint8_t reg_addr, uint16_t reg_data,
                                     struct rtio_sqe **out);

int ina219_reg_read_rtio(const struct ina219_bus *bus, uint8_t reg_addr,
                         uint8_t *raw_reg_data);

int ina219_reg_write_rtio(const struct ina219_bus *bus, uint8_t reg_addr,
                          uint16_t reg_data);

#endif // SENSOR_INA219_ASYNC_BUS_H
