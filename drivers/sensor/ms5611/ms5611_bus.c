#include "ms5611_bus.h"
#include "zephyr/rtio/sqe.h"

#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>

// Consumes all cqe events, returns the first non-zero return code.
static int consume_cqe(struct rtio *ctx) {
  int ret = 0;

  struct rtio_cqe *cqe = rtio_cqe_consume(ctx);
  while (cqe != NULL) {
    if (ret == 0) {
      ret = cqe->result;
    }
    rtio_cqe_release(ctx, cqe);

    cqe = rtio_cqe_consume(ctx);
  }

  return ret;
}

// Creates a 2 sqe sequence of write + read which are part of a transaction, so
// are execute as one.
// The return value, if positive, signals the number of sqe created.
//
// Outputs 2 bytes.
//
// NOTE: the values are raw MSB first.
int ms5611_prep_PROM_read_rtio_async(const struct ms5611_bus *bus,
                                     uint8_t offset, uint16_t *rx_buf,
                                     struct rtio_sqe **out) {
  struct rtio_sqe *write_sqe = rtio_sqe_acquire(bus->rtio.ctx);
  struct rtio_sqe *read_sqe = rtio_sqe_acquire(bus->rtio.ctx);
  if (!write_sqe || !read_sqe) {
    rtio_sqe_drop_all(bus->rtio.ctx);
    return -ENOMEM;
  }

  // This two operation are executed as one, because of the RTIO_SQE_TRANSACTION
  // flag (i.e. the CS remains asserted and the read begins after the write,
  // without missing a single SCLK beat).
  uint8_t command = COMMAND_PROM_READ | (offset << 1);
  rtio_sqe_prep_tiny_write(write_sqe, bus->rtio.iodev, RTIO_PRIO_NORM, &command,
                           1, NULL);
  write_sqe->flags |= RTIO_SQE_TRANSACTION;

  rtio_sqe_prep_read(read_sqe, bus->rtio.iodev, RTIO_PRIO_NORM,
                     (uint8_t *)rx_buf, 2, NULL);

  if (out) {
    *out = read_sqe;
  }

  return 2;
}

// See `ms5611_prep_PROM_read_rtio_async(...)`.
//
// Outputs 3 bytes.
//
// NOTE: the values are raw MSB first.
int ms5611_prep_ADC_read_rtio_async(const struct ms5611_bus *bus,
                                    uint8_t *rx_buf, struct rtio_sqe **out) {

  struct rtio_sqe *write_sqe = rtio_sqe_acquire(bus->rtio.ctx);
  struct rtio_sqe *read_sqe = rtio_sqe_acquire(bus->rtio.ctx);
  if (!write_sqe || !read_sqe) {
    rtio_sqe_drop_all(bus->rtio.ctx);
    return -ENOMEM;
  }

  uint8_t command = COMMAND_ADC_READ;
  rtio_sqe_prep_tiny_write(write_sqe, bus->rtio.iodev, RTIO_PRIO_NORM, &command,
                           1, NULL);
  write_sqe->flags |= RTIO_SQE_TRANSACTION;

  rtio_sqe_prep_read(read_sqe, bus->rtio.iodev, RTIO_PRIO_NORM, rx_buf, 3,
                     NULL);

  if (out) {
    *out = read_sqe;
  }

  return 2;
}

// See `ms5611_prep_PROM_read_rtio_async(...)`.
int ms5611_prep_command_write_rtio_async(const struct ms5611_bus *bus,
                                         uint8_t command,
                                         struct rtio_sqe **out) {

  struct rtio_sqe *write_sqe = rtio_sqe_acquire(bus->rtio.ctx);
  if (!write_sqe) {
    rtio_sqe_drop_all(bus->rtio.ctx);
    return -ENOMEM;
  }

  rtio_sqe_prep_tiny_write(write_sqe, bus->rtio.iodev, RTIO_PRIO_NORM, &command,
                           1, NULL);

  if (out) {
    *out = write_sqe;
  }

  return 1;
}

// Sync version of the PROM read. Essentially submits and blocks the rtio
// sequence of operation, so there's a little bit of overhead that we accept for
// better maintenability.
int ms5611_PROM_read_rtio(const struct ms5611_bus *bus, uint8_t offset,
                          uint16_t *rx_buf) {
  int ret = ms5611_prep_PROM_read_rtio_async(bus, offset, rx_buf, NULL);
  if (ret < 0) {
    return ret;
  }

  ret = rtio_submit(bus->rtio.ctx, ret);
  if (ret) {
    return ret;
  }

  ret = consume_cqe(bus->rtio.ctx);

  return ret;
}

// See `ms5611_ADC_read_rtio(...)`.
int ms5611_ADC_read_rtio(const struct ms5611_bus *bus, uint8_t *rx_buf) {
  int ret = ms5611_prep_ADC_read_rtio_async(bus, rx_buf, NULL);
  if (ret < 0) {
    return ret;
  }

  ret = rtio_submit(bus->rtio.ctx, ret);
  if (ret) {
    return ret;
  }

  ret = consume_cqe(bus->rtio.ctx);

  return ret;
}

// See `ms5611_ADC_read_rtio(...)`.
int ms5611_command_write_rtio(const struct ms5611_bus *bus, uint8_t command) {
  int ret = ms5611_prep_command_write_rtio_async(bus, command, NULL);
  if (ret < 0) {
    return ret;
  }

  ret = rtio_submit(bus->rtio.ctx, ret);
  if (ret) {
    return ret;
  }

  ret = consume_cqe(bus->rtio.ctx);

  return ret;
}
