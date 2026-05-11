/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _GNSS_U_BLOX_M10_I2C_H_
#define _GNSS_U_BLOX_M10_I2C_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/modem/pipe.h>

#define M10_I2C_ADDR 0x42

#define M10_REG_BYTES_AVAIL_H 0xFD
#define M10_REG_BYTES_AVAIL_L 0xFE
#define M10_REG_DATA         0xFF

struct gnss_u_blox_m10_i2c_config {
    struct i2c_dt_spec i2c;
    uint32_t fix_rate_ms;
};

int gnss_u_blox_m10_i2c_init(struct modem_pipe *pipe,
                         const struct gnss_u_blox_m10_i2c_config *cfg,
                         void *user_data);

#endif /* _GNSS_U_BLOX_M10_I2C_H_ */