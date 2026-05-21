#include "ina219_bus.h"

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

// Returns 2 raw bytes as raw MSB first values
int ina219_prep_reg_read_rtio_async(const struct ina219_bus *bus,
                                    uint8_t reg_addr, uint8_t *raw_reg_data,
                                    struct rtio_sqe **out) {

  struct rtio *ctx = bus->rtio.ctx;
  struct rtio_iodev *iodev = bus->rtio.iodev;
  struct rtio_sqe *write_sqe = rtio_sqe_acquire(ctx);
  struct rtio_sqe *read_sqe = rtio_sqe_acquire(ctx);

  if (!write_sqe || !read_sqe) {
    rtio_sqe_drop_all(ctx);
    return -ENOMEM;
  }

  rtio_sqe_prep_tiny_write(write_sqe, iodev, RTIO_PRIO_NORM, &reg_addr, 1,
                           NULL);
  write_sqe->flags |= RTIO_SQE_TRANSACTION;
  rtio_sqe_prep_read(read_sqe, iodev, RTIO_PRIO_NORM, raw_reg_data, 2, NULL);
  read_sqe->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;

  if (out) {
    *out = read_sqe;
  }

  return 2;
}

int ina219_prep_reg_write_rtio_async(const struct ina219_bus *bus,
                                     uint8_t reg_addr, uint16_t reg_data,
                                     struct rtio_sqe **out) {

  struct rtio_sqe *write_reg_sqe = rtio_sqe_acquire(bus->rtio.ctx);
  struct rtio_sqe *write_data_sqe = rtio_sqe_acquire(bus->rtio.ctx);

  if (!write_reg_sqe || !write_data_sqe) {
    rtio_sqe_drop_all(bus->rtio.ctx);
    return -ENOMEM;
  }

  rtio_sqe_prep_tiny_write(write_reg_sqe, bus->rtio.iodev, RTIO_PRIO_NORM,
                           &reg_addr, 1, NULL);
  write_reg_sqe->flags |= RTIO_SQE_TRANSACTION;
  rtio_sqe_prep_tiny_write(write_data_sqe, bus->rtio.iodev, RTIO_PRIO_NORM,
                           (uint8_t *)&reg_data, 2, NULL);
  write_data_sqe->iodev_flags |= RTIO_IODEV_I2C_STOP;

  if (out) {
    *out = write_data_sqe;
  }

  return 2;
}

int ina219_reg_read_rtio(const struct ina219_bus *bus, uint8_t reg_addr,
                         uint8_t *raw_reg_data) {

  int ret = ina219_prep_reg_read_rtio_async(bus, reg_addr, raw_reg_data, NULL);
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

int ina219_reg_write_rtio(const struct ina219_bus *bus, uint8_t reg_addr,
                          uint16_t reg_data) {

  int ret = ina219_prep_reg_write_rtio_async(bus, reg_addr, reg_data, NULL);
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
