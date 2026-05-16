# MS5611 Barometric Pressure Sensor Driver

## Sensor Overview

The MS5611 is a high-resolution barometric pressure sensor from Measurement Specialties. It provides digital output of pressure and temperature data with 24-bit ADC resolution.

### Key Features

- **Pressure range**: 10-1200 mbar (1000-120000 Pa)
- **Pressure resolution**: 0.012 mbar (programmable)
- **Temperature resolution**: 0.012°C
- **I2C and SPI interface** (this driver uses SPI)
- **Factory calibrated** with 6 coefficients stored in internal PROM

### Measurement Process

The MS5611 uses a two-step measurement process:
1. **D1 conversion**: Read raw pressure value (requires ~10ms conversion time)
2. **D2 conversion**: Read raw temperature value (requires ~10ms conversion time)

The raw ADC values are then compensated using factory-calibrated PROM coefficients to produce accurate pressure (mBar) and temperature (°C) readings.

---

## RTIO Async Sensor Architecture

This driver implements Zephyr's async sensor API using RTIO (Real-Time I/O) for non-blocking operations.

### How It Works

The async sensor API adds two optional operations to the standard sensor driver API:

```c
static DEVICE_API(sensor, ms5611_driver_api) = {
    .sample_fetch = ms5611_sample_fetch,  // Sync fallback
    .channel_get = ms5611_channel_get,
    .attr_set = ms5611_attr_set,
    .submit = ms5611_submit,              // Async submit handler
    .get_decoder = ms5611_get_decoder,    // Decoder for data format
};
```

### Data Flow

1. **Application call**: `sensor_read()` creates an RTIO request
2. **Submit callback**: Driver's `submit` function receives the `rtio_iodev_sqe`
3. **Build operation chain**: Driver chains SPI transfers + delays + completion callback
4. **RTIO executor**: Processes the chain asynchronously (non-blocking)
5. **Completion**: Driver signals success via `rtio_iodev_sqe_ok()` or error via `rtio_iodev_sqe_err()`

### The Decoder

The decoder converts the driver's internal data format to the standard sensor Q31 format:

```c
struct sensor_decoder_api {
    int (*get_frame_count)(...);
    int (*get_size_info)(...);
    int (*decode)(...);  // Converts raw data to Q31 fixed-point
};
```

Key design: The callback is a simple pass-through. All compensation (raw ADC → temperature/pressure) happens in the decoder, maintaining separation of concerns.

---

## Key Implementation Details

### Encoded Data Structure

```c
struct ms5611_encoded_data {
    struct ms5611_data *sensor_data;  // Pointer for PROM coefficients
    struct { uint64_t timestamp; } header;
    struct { uint8_t has_temp : 1; uint8_t has_press : 1; } flags;
    uint8_t rx_d1[3];  // Raw ADC for pressure (D1)
    uint8_t rx_d2[3];  // Raw ADC for temperature (D2)
} __attribute__((__packed__));
```

### RTIO Context Instantiation

From ms5611.c lines 417-431:

```
SPI_DT_IODEV_DEFINE creates the device for the RTIO context.

RTIO_DEFINE creates the rtio context, by specifying the sqe and cqe number.
  - The sqe number should be enough to contain the maximum number of sqe
    operations in a single "operation". Otherwise the device would return
    -ENOMEM.
  - The cqe number is not relevant, as completion event area automatically
    used to chain sqes, and the cqe which concludes operations is forwarded
    to the RTIO_CONTEXT of the caller of the sensor operations (the overflow
    should be automatically handled by the system).

The RTIO context is instantiated even if the SENSOR_ASYNC_API is disabled, as
it enables more code reusability at the cost of a minimal overhead.
```

### SPI Operation Chain

The MS5611 requires this sequence:
1. Write CONVERT_D1 → delay ~10ms → read ADC (D1)
2. Write CONVERT_D2 → delay ~10ms → read ADC (D2)

Each operation is chained using `RTIO_SQE_CHAINED` flag.

### Important Note: Compensation Dependency

The MS5611 pressure calculation requires temperature data (the `dt` value). This means:
- If only **pressure** is requested, the driver still reads both D1 and D2
- The decoder uses both to compute compensated pressure
- Temperature can be fetched independently

---

## Configuration

Enable async sensor API in your `prj.conf`:

```
CONFIG_SENSOR_ASYNC_API=y
CONFIG_RTIO=y
CONFIG_RTIO_OP_DELAY=y
CONFIG_SPI_RTIO=y
```

The driver itself is enabled via:
```
CONFIG_SENSOR_MS5611_ASYNC=y
```

---

## API Usage

### Synchronous (blocking)
```c
sensor_sample_fetch(dev, SENSOR_CHAN_ALL);
sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
```

### Asynchronous (non-blocking with RTIO)
```c
struct sensor_read_config cfg = {
    .channels = {{.chan_type = SENSOR_CHAN_ALL, .chan_idx = 0}},
    .count = 1,
};
sensor_read(dev, &cfg, callback, user_data);
```