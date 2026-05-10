# Guide: Converting a Sync Sensor Driver to Async RTIO

This guide walks through the process of adding async RTIO support to an existing synchronous sensor driver, using the MS5611 barometric pressure sensor as a real-world example.

---

## Prerequisites

Understanding of:
- Zephyr device driver model (device, config, data structures)
- SPI communication basics
- RTIO subsystem concepts

Required Kconfig options:
- `CONFIG_SENSOR_ASYNC_API` - Core async sensor support
- `CONFIG_RTIO` - RTIO subsystem  
- `CONFIG_RTIO_OP_DELAY` - For delay SQE support
- `CONFIG_SPI_RTIO` - SPI RTIO operations

---

## Part 1: Understanding the Sync Driver

A typical Zephyr sync sensor driver implements:
- `sample_fetch()` - Reads sensor data synchronously
- `channel_get()` - Returns sensor values to the application

The MS5611 sync driver uses blocking SPI calls:
```c
// ms5611.c (lines 197-229)
static int ms5611_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    // Write conversion command
    ret = ms5611_write(&conf->spi, COMMAND_CONVERT_D1);
    k_sleep(K_MSEC(10));  // Wait for conversion
    ret = ms5611_read_ADC(&conf->spi, &rawPressure);
    
    // Repeat for temperature...
    ms5611_compensate(rawPressure, rawTemperature, data);
    return 0;
}
```

The problem: These are blocking calls that monopolize the CPU during SPI transactions and delays.

---

## Part 2: Key Async Concepts

### The Sensor Async API

The async sensor API adds two optional operations to the driver API:

```c
static DEVICE_API(sensor, ms5611_driver_api) = {
    .sample_fetch = ms5611_sample_fetch,  // Still needed for sync fallback
    .channel_get = ms5611_channel_get,
    .attr_set = ms5611_attr_set,
#ifdef CONFIG_SENSOR_MS5611_ASYNC
    .submit = ms5611_submit,               // NEW: Async submit handler
    .get_decoder = ms5611_get_decoder,    // NEW: Decoder for data format
#endif
};
```

### How It Works

1. Application calls `sensor_read()` which creates an RTIO request
2. Driver's `submit` callback is invoked with an `rtio_iodev_sqe`
3. Driver builds a chain of RTIO operations (SPI transfers, delays, callback)
4. RTIO executor processes the chain asynchronously
5. When complete, driver signals `rtio_iodev_sqe_ok()` or `rtio_iodev_sqe_err()`

### Decoder API

The decoder converts the driver's internal data format to the standard sensor data types (q31, float, etc.):

```c
struct sensor_decoder_api {
    int (*get_frame_count)(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
                           uint16_t *frame_count);
    int (*get_size_info)(struct sensor_chan_spec channel, size_t *base_size,
                         size_t *frame_size);
    int (*decode)(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
                  uint32_t *fit, uint16_t max_count, void *data_out);
};
```

---

## Part 3: Implementation Steps

### Step 1: Add Async Headers and SPI Operation Macro

```c
// ms5611.c (lines 9-13, 32)
#ifdef CONFIG_SENSOR_MS5611_ASYNC
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/rtio/rtio.h>
#endif

#define MS5611_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)
```

### Step 2: Define Encoded Data Structure

The encoded data structure is what gets passed through RTIO. It should contain:
- Timestamp
- Flags indicating which channels were requested
- The actual sensor data (raw or processed)
- Any buffers needed for SPI operations

```c
// ms5611.c (lines 34-49)
struct ms5611_encoded_data {
    struct ms5611_data *sensor_data;  // Pointer to device data (for coeffs)
    struct { uint64_t timestamp; } header;
    struct { uint8_t has_temp : 1; uint8_t has_press : 1; } flags;
    struct { int32_t temp; uint32_t press; } reading;  // Compensated values
    uint8_t rx_d1[4];  // SPI receive buffer for pressure
    uint8_t rx_d2[4];  // SPI receive buffer for temperature
} __attribute__((__packed__));
```

**Key insight**: Embed the SPI rx buffers directly in the encoded data structure. This ensures they live in the right memory location for the callback to access them.

### Step 3: Add RTIO Fields to Data Structure

Add `rtio_ctx` and `iodev` pointers to the data structure (not config):

```c
// ms5611.c (lines 53-60)
struct ms5611_data {
    uint16_t coeffs[6];
    struct ms5611_sample last_sample;
#ifdef CONFIG_SENSOR_MS5611_ASYNC
    struct rtio *rtio_ctx;
    struct rtio_iodev *iodev;
#endif
};
```

**Why data, not config?**
- Config is typically `const` and initialized at compile time
- Data holds runtime state that may be modified
- Multiple sensor instances each need their own RTIO context

### Step 4: Define Device Instantiation Macro

This is crucial - you need to create the RTIO resources for each device instance:

```c
// ms5611.c (lines 559-576)
#ifdef CONFIG_SENSOR_MS5611_ASYNC
#define MS5611_RTIO_DEFINE(inst) \
    SPI_DT_IODEV_DEFINE(ms5611_iodev_##inst, DT_DRV_INST(inst), \
                        MS5611_SPI_OPERATION); \
    RTIO_DEFINE(ms5611_rtio_ctx_##inst, 8, 8);

#define MS5611_INIT(inst) \
    MS5611_RTIO_DEFINE(inst); \
    static struct ms5611_data ms5611_data_##inst = { \
        .rtio_ctx = &ms5611_rtio_ctx_##inst, \
        .iodev = &ms5611_iodev_##inst, \
    }; \
    static const struct ms5611_config ms5611_config_##inst = { \
        .spi = SPI_DT_SPEC_INST_GET(inst, MS5611_SPI_OPERATION), \
    }; \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, ms5611_init, NULL, \
                                 &ms5611_data_##inst, &ms5611_config_##inst, \
                                 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                                 &ms5611_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MS5611_INIT)
#else
// Non-async version (sync only)
#define MS5611_INIT(inst) ...
DT_INST_FOREACH_STATUS_OKAY(MS5611_INIT)
#endif
```

This creates for each instance:
1. `SPI_DT_IODEV_DEFINE` - Creates the SPI RTIO I/O device
2. `RTIO_DEFINE` - Creates the RTIO context (submission/completion queues)
3. Initializes the data structure with pointers to these resources

### Step 5: Implement the Submit Callback

The submit callback is where the magic happens. It receives the RTIO request and builds a chain of operations.

```c
// ms5611.c (lines 388-545)
static void ms5611_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe) {
    // 1. Get channel configuration
    const struct sensor_read_config *read_cfg = iodev_sqe->sqe.iodev->data;
    
    // 2. Validate requested channels
    for (size_t i = 0; i < read_cfg->count; i++) {
        switch (read_cfg->channels[i].chan_type) {
        case SENSOR_CHAN_AMBIENT_TEMP:
        case SENSOR_CHAN_PRESS:
        case SENSOR_CHAN_ALL:
            break;
        default:
            rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
            return;
        }
    }
    
    // 3. Get RTIO buffer for encoded data
    struct ms5611_encoded_data *edata;
    uint32_t buf_len;
    int err = rtio_sqe_rx_buf(iodev_sqe, sizeof(*edata), sizeof(*edata),
                              (uint8_t **)&edata, &buf_len);
    if (err < 0) {
        rtio_iodev_sqe_err(iodev_sqe, err);
        return;
    }
    
    // 4. Initialize flags based on requested channels
    edata->flags.has_temp = 0;
    edata->flags.has_press = 0;
    for (size_t i = 0; i < read_cfg->count; i++) {
        switch (read_cfg->channels[i].chan_type) {
        case SENSOR_CHAN_AMBIENT_TEMP: edata->flags.has_temp = 1; break;
        case SENSOR_CHAN_PRESS: edata->flags.has_press = 1; break;
        case SENSOR_CHAN_ALL: edata->flags.has_temp = edata->flags.has_press = 1; break;
        default: break;
        }
    }
    
    // 5. Get hardware timestamp
    uint64_t cycles;
    err = sensor_clock_get_cycles(&cycles);
    edata->header.timestamp = sensor_clock_cycles_to_ns(cycles);
    edata->sensor_data = dev->data;  // Store pointer to access coeffs later
    
    // 6. Build SPI operation chain
    struct rtio *ctx = data->rtio_ctx;
    struct rtio_iodev *iodev = data->iodev;
    struct rtio_sqe *last_sqe;
    
    // D1: Write convert command
    err = spi_rtio_copy(ctx, iodev, &tx_d1_conv_set, NULL, &last_sqe);
    last_sqe->flags |= RTIO_SQE_CHAINED;
    
    // D1: Delay for conversion
    last_sqe = rtio_sqe_acquire(ctx);
    rtio_sqe_prep_delay(last_sqe, K_MSEC(10), NULL);
    last_sqe->flags |= RTIO_SQE_CHAINED;
    
    // D1: Read ADC result
    err = spi_rtio_copy(ctx, iodev, &tx_d1_read_set, &rx_d1_set, &last_sqe);
    last_sqe->flags |= RTIO_SQE_CHAINED;
    
    // Repeat for D2 (temperature)...
    
    // 7. Add completion callback
    struct rtio_sqe *complete_sqe = rtio_sqe_acquire(ctx);
    rtio_sqe_prep_callback(complete_sqe, ms5611_complete_result, NULL, NULL);
    
    // 8. Submit the chain
    rtio_submit(ctx, 0);
}
```

### Step 6: Implement the Callback

The callback receives the result. With `rtio_sqe_prep_callback()`, the callback receives the result directly without needing to manually consume CQEs:

```c
// ms5611.c (lines 352-386)
static void ms5611_complete_result(struct rtio *ctx, const struct rtio_sqe *sqe,
                                   int result, void *arg) {
    // rtio_sqe and rtio_iodev_sqe can be safely cast to each other
    // (rtio_iodev_sqe has rtio_sqe as its first member)
    struct rtio_iodev_sqe *iodev_sqe = (struct rtio_iodev_sqe *)sqe;
    
    if (result < 0) {
        rtio_iodev_sqe_err(iodev_sqe, result);
        return;
    }
    
    // In this implementation, compensation is done in the decoder (see below)
    // But you could also do it here:
    // struct ms5611_encoded_data *edata = iodev_sqe->sqe.rx.buf;
    // uint16_t *coeffs = edata->sensor_data->coeffs;
    // ...do compensation...
    
    rtio_iodev_sqe_ok(iodev_sqe, 0);
}
```

### Step 7: Implement the Decoder

The decoder converts the encoded data to the standard q31 format:

```c
// ms5611.c (lines 262-343)
static int ms5611_decoder_get_frame_count(const uint8_t *buffer,
                                          struct sensor_chan_spec chan_spec,
                                          uint16_t *frame_count) {
    const struct ms5611_encoded_data *edata = (const struct ms5611_encoded_data *)buffer;
    
    switch (chan_spec.chan_type) {
    case SENSOR_CHAN_AMBIENT_TEMP:
        *frame_count = edata->flags.has_temp ? 1 : 0;
        break;
    case SENSOR_CHAN_PRESS:
        *frame_count = edata->flags.has_press ? 1 : 0;
        break;
    default:
        return -ENOTSUP;
    }
    return *frame_count > 0 ? 0 : -ENODATA;
}

static int ms5611_decoder_get_size_info(struct sensor_chan_spec chan_spec,
                                         size_t *base_size, size_t *frame_size) {
    *base_size = sizeof(struct sensor_q31_data);
    *frame_size = sizeof(struct sensor_q31_sample_data);
    return 0;
}

static int ms5611_decoder_decode(const uint8_t *buffer,
                                  struct sensor_chan_spec chan_spec,
                                  uint32_t *fit, uint16_t max_count,
                                  void *data_out) {
    const struct ms5611_encoded_data *edata = (const struct ms5611_encoded_data *)buffer;
    struct sensor_q31_data *out = data_out;
    
    out->header.base_timestamp_ns = edata->header.timestamp;
    out->header.reading_count = 1;
    out->shift = -15;  // Values are in centi-units, divide by 100
    
    switch (chan_spec.chan_type) {
    case SENSOR_CHAN_AMBIENT_TEMP:
        if (edata->flags.has_temp) {
            out->readings[0].value = (edata->reading.temp / 100);
        } else {
            return -ENODATA;
        }
        break;
    case SENSOR_CHAN_PRESS:
        if (edata->flags.has_press) {
            out->readings[0].value = (edata->reading.press / 100);
        } else {
            return -ENODATA;
        }
        break;
    default:
        return -EINVAL;
    }
    
    *fit = 1;
    return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
    .get_frame_count = ms5611_decoder_get_frame_count,
    .get_size_info = ms5611_decoder_get_size_info,
    .decode = ms5611_decoder_decode,
};

int ms5611_get_decoder(const struct device *dev,
                       const struct sensor_decoder_api **decoder) {
    ARG_UNUSED(dev);
    *decoder = &SENSOR_DECODER_NAME();
    return 0;
}
```

---

## Part 4: Key Implementation Details

### SPI Chain Pattern

The MS5611 requires a specific sequence:
1. Write CONVERT_D1 → wait ~10ms → read ADC
2. Write CONVERT_D2 → wait ~10ms → read ADC

Each SPI operation needs separate buffers because:
- Convert commands are WRITE-only (null tx, no rx needed)
- ADC reads are TRANSCEIVE (write ADC_READ command, read result)

```c
// Write-only operation (no rx buffer)
err = spi_rtio_copy(ctx, iodev, &tx_conv_set, NULL, &last_sqe);

// Transceive operation (both tx and rx)
err = spi_rtio_copy(ctx, iodev, &tx_read_set, &rx_set, &last_sqe);
```

### RTIO_SQE_CHAINED Flag

Setting `RTIO_SQE_CHAINED` on an SQE tells RTIO there's another operation following it in the chain. The flag is set on the **current** operation to indicate it has a **next** operation.

### Timestamp with sensor_clock

The `sensor_clock_get_cycles()` and `sensor_clock_cycles_to_ns()` functions provide hardware timestamps tied to the sensor's clock domain, essential for accurate multi-sensor synchronization.

### Error Handling

Key error handling patterns:
- If any SPI operation fails, call `rtio_sqe_drop_all(ctx)` to clean up and `rtio_iodev_sqe_err()` to signal error
- The callback receives the error result directly with `rtio_sqe_prep_callback()`

---

## Part 5: Kconfig Dependencies

```kconfig
# Kconfig (in driver directory)
config SENSOR_MS5611_ASYNC
    bool "MS5611 async API support"
    depends on SENSOR_ASYNC_API && RTIO_OP_DELAY && SPI_RTIO
    default y
```

Required Kconfig options (must be enabled in the build):
- `CONFIG_SENSOR_ASYNC_API`
- `CONFIG_RTIO`
- `CONFIG_RTIO_OP_DELAY`
- `CONFIG_SPI_RTIO`

---

## Summary

Converting a sync sensor to async RTIO involves:

1. **Data structures**: Add RTIO context to data, define encoded data with SPI buffers
2. **Instantiation**: Create RTIO resources per device instance with macros
3. **Submit callback**: Build SPI operation chain, get timestamp, request buffer
4. **Callback**: Signal completion (optionally do compensation here)
5. **Decoder**: Convert encoded data to q31 format (do compensation here if preferred)

The key benefits:
- Non-blocking SPI operations - CPU can do other work during SPI transfers
- Hardware timestamps from sensor clock
- Standard decoder API for data format conversion
- Multiple sensor instances handled automatically by DT macros