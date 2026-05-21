/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/sys/byteorder.h>

#include "gnss_u_blox_m10_i2c.h"

LOG_MODULE_REGISTER(gnss_u_blox_m10_i2c, CONFIG_GNSS_LOG_LEVEL);

static int m10_i2c_open(void *data)
{
    ARG_UNUSED(data);
    return 0;
}

static int m10_i2c_transmit(void *data, const uint8_t *buf, size_t size)
{
    const struct gnss_u_blox_m10_i2c_config *cfg = data;
    int ret;

    ret = i2c_write_dt(&cfg->i2c, buf, size);
    if (ret < 0) {
        LOG_ERR("I2C write failed: %d", ret);
        return ret;
    }

    return 0;
}

static int m10_i2c_receive(void *data, uint8_t *buf, size_t size)
{
    const struct gnss_u_blox_m10_i2c_config *cfg = data;
    int ret;
    uint8_t avail_buf[2];
    uint16_t avail;
    uint8_t reg;

    reg = M10_REG_BYTES_AVAIL_H;
    ret = i2c_write_dt(&cfg->i2c, &reg, 1);
    if (ret < 0) {
        return ret;
    }

    ret = i2c_read_dt(&cfg->i2c, avail_buf, sizeof(avail_buf));
    if (ret < 0) {
        return ret;
    }

    avail = sys_be16_to_cpu((avail_buf[0] << 8) | avail_buf[1]);

    if (avail == 0) {
        return -EAGAIN;
    }

    if (avail > size) {
        avail = size;
    }

    reg = M10_REG_DATA;
    ret = i2c_write_dt(&cfg->i2c, &reg, 1);
    if (ret < 0) {
        return ret;
    }

    ret = i2c_read_dt(&cfg->i2c, buf, avail);
    if (ret < 0) {
        return ret;
    }

    LOG_DBG("Read %d bytes from M10", ret);
    return ret;
}

static int m10_i2c_close(void *data)
{
    ARG_UNUSED(data);
    return 0;
}

static const struct modem_pipe_api m10_i2c_pipe_api = {
    .open = m10_i2c_open,
    .transmit = m10_i2c_transmit,
    .receive = m10_i2c_receive,
    .close = m10_i2c_close,
};

int gnss_u_blox_m10_i2c_init(struct modem_pipe *pipe,
                         const struct gnss_u_blox_m10_i2c_config *cfg,
                         void *user_data)
{
    ARG_UNUSED(user_data);
    modem_pipe_init(pipe, (void *)cfg, &m10_i2c_pipe_api);
    return 0;
}