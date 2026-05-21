#ifndef SENSOR_MS5611_BUS_H_
#define SENSOR_MS5611_BUS_H_

#include <zephyr/rtio/sqe.h>

#include <stddef.h>
#include <stdint.h>

#define COMMAND_PROM_READ 0xA0
#define COMMAND_ADC_READ 0x00
#define COMMAND_CONVERT_D1 0x40
#define COMMAND_CONVERT_D2 0x50
#define COMMAND_RESET 0x1E

struct ms5611_bus {
  struct {
    struct rtio *ctx;
    struct rtio_iodev *iodev;
  } rtio;
};

int ms5611_prep_PROM_read_rtio_async(const struct ms5611_bus *bus,
                                     uint8_t offset, uint16_t *rx_buf,
                                     struct rtio_sqe **out);

int ms5611_prep_ADC_read_rtio_async(const struct ms5611_bus *bus,
                                    uint8_t *rx_buf, struct rtio_sqe **out);

int ms5611_prep_command_write_rtio_async(const struct ms5611_bus *bus,
                                         uint8_t command,
                                         struct rtio_sqe **out);

int ms5611_PROM_read_rtio(const struct ms5611_bus *bus, uint8_t offset,
                          uint16_t *rx_buf);

int ms5611_ADC_read_rtio(const struct ms5611_bus *bus, uint8_t *rx_buf);

int ms5611_command_write_rtio(const struct ms5611_bus *bus, uint8_t command);

#endif // SENSOR_MS5611_BUS_H_
