#ifndef SENSOR_MS5611_H_
#define SENSOR_MS5611_H_

#include "ms5611_bus.h"

#include <zephyr/drivers/spi.h>

#include <stdint.h>

#define DT_DRV_COMPAT ams_ms5611

#define MS5611_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
#define MS5611_MEASUREMENT_DELAY_MS 10
#define MS5611_RESET_DELAY_MS 3

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
  uint8_t rx_d1[3];
  uint8_t rx_d2[3];
} __attribute__((__packed__));

#endif // CONFIG_SENSOR_MS5611_ASYNC

// Defines the runtime state of the sensor
struct ms5611_data {
  uint16_t coeffs[6];
  struct ms5611_sample last_sample;
};

// Defines the static configuration of the sensor
struct ms5611_config {
  struct spi_dt_spec spi;
  struct ms5611_bus bus;
};

#endif // SENSOR_MS5611_H_
