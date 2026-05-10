#ifndef SENSOR_MS5611_H_
#define SENSOR_MS5611_H_

#include <zephyr/drivers/spi.h>

#include <stdint.h>

#define DT_DRV_COMPAT ams_ms5611

#define MS5611_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
#define MS5611_MEASUREMENT_DELAY_MS 10

#define COMMAND_PROM_READ 0xA0
#define COMMAND_ADC_READ 0x00
#define COMMAND_CONVERT_D1 0x40
#define COMMAND_CONVERT_D2 0x50
#define COMMAND_RESET 0x1E

struct ms5611_sample {
  int32_t pressure;
  int32_t temperature;
};

#ifdef CONFIG_SENSOR_MS5611_ASYNC

// Defines the RTIO buffer format
struct ms5611_encoded_data {
  struct ms5611_data *sensor_data;
  struct {
    uint64_t timestamp; // in nanoseconds
  } header;
  struct {
    uint8_t has_temp : 1;
    uint8_t has_press : 1;
  } flags;
  uint8_t rx_d1[4];
  uint8_t rx_d2[4];
} __attribute__((__packed__));

#endif // CONFIG_SENSOR_MS5611_ASYNC

// Defines the runtime state of the sensor
struct ms5611_data {
  uint16_t coeffs[6];
  struct ms5611_sample last_sample;
#ifdef CONFIG_SENSOR_MS5611_ASYNC
  struct rtio *rtio_ctx;
  struct rtio_iodev *iodev;
#endif
};

// Defines the static configuration of the sensor
struct ms5611_config {
  struct spi_dt_spec spi;
};

#endif // SENSOR_MS5611_H_
