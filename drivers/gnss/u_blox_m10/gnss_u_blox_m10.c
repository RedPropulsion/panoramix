/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * u-blox M10 GNSS driver over I2C with non-blocking API
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "gnss_u_blox_m10_i2c.h"
#include "gnss_u_blox_m10.h"

LOG_MODULE_REGISTER(gnss_u_blox_m10, CONFIG_GNSS_LOG_LEVEL);

#define M10_GNSS_NODE DT_NODELABEL(gps)

#define UBX_SYNC_1 0xB5
#define UBX_SYNC_2 0x62
#define UBX_CLASS_NAV 0x01
#define UBX_MSG_NAV_PVT 0x07

#define RING_BUFFER_SIZE 2

struct m10_data {
    struct gps_position ring_buffer[RING_BUFFER_SIZE];
    atomic_t write_idx;
    atomic_t read_idx;
    uint8_t parse_buf[256];
    size_t parse_len;
    bool awaiting_sync;
};

static struct m10_data m10_data_instance;

static const struct gnss_u_blox_m10_i2c_config m10_i2c_config = {
    .i2c = I2C_DT_SPEC_GET(M10_GNSS_NODE),
    .fix_rate_ms = 100, //TODO: make this configurable via Kconfig?
};

static int m10_send_cfg(const struct i2c_dt_spec *i2c, uint32_t key_id, uint32_t value)
{
    uint8_t cfg_msg[] = {
        0xB5, 0x62,             /* UBX sync chars */
        0x06, 0x8A,             /* CFG-VALSET */
        0x04, 0x00,             /* Payload length (4 bytes) */
        0x00, 0x00, 0x00, 0x00, /* Key ID (will fill) */
        0x00, 0x00, 0x00, 0x00, /* Value (will fill) */
        0x00, 0x00              /* Checksum placeholder */
    };

    cfg_msg[8] = (key_id >> 0) & 0xFF;
    cfg_msg[9] = (key_id >> 8) & 0xFF;
    cfg_msg[10] = (key_id >> 16) & 0xFF;
    cfg_msg[11] = (key_id >> 24) & 0xFF;

    cfg_msg[12] = (value >> 0) & 0xFF;
    cfg_msg[13] = (value >> 8) & 0xFF;
    cfg_msg[14] = (value >> 16) & 0xFF;
    cfg_msg[15] = (value >> 24) & 0xFF;

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 16; i++) {
        ck_a += cfg_msg[i];
        ck_b += ck_a;
    }
    cfg_msg[16] = ck_a;
    cfg_msg[17] = ck_b;

    return i2c_write_dt(i2c, cfg_msg, 18);
}

static int m10_configure(const struct device *dev)
{
    ARG_UNUSED(dev);
    const struct i2c_dt_spec *i2c = &m10_i2c_config.i2c;
    int ret;

    LOG_INF("Configuring M10 for 10Hz UBX-NAV-PVT on I2C");

    /* Set measurement rate to 100ms (10Hz) */
    ret = m10_send_cfg(i2c, 0x30210001, 100);
    if (ret < 0) {
        LOG_ERR("Failed to set rate: %d", ret);
        return ret;
    }
    k_sleep(K_MSEC(10));

    /* Enable UBX-NAV-PVT on I2C */
    ret = m10_send_cfg(i2c, 0x20910007, 1);
    if (ret < 0) {
        LOG_ERR("Failed to enable NAV-PVT: %d", ret);
        return ret;
    }
    k_sleep(K_MSEC(10));

    /* Disable NMEA on I2C to reduce traffic */
    ret = m10_send_cfg(i2c, 0x10740002, 0);
    if (ret < 0) {
        LOG_WRN("Failed to disable NMEA: %d", ret);
    }
    k_sleep(K_MSEC(10));

    LOG_INF("M10 configuration complete");
    return 0;
}

static int m10_parse_ubx_nav_pvt(struct gps_position *pos, const uint8_t *data, size_t len)
{
    if (len < 92) {
        return -EINVAL;
    }

    const uint8_t *p = data;

    /* Skip: itow(4), year(2), month(1), day(1), hour(1), minute(1), second(1), valid(1), tacc(4), nano(4) */
    p += 20;

    pos->fix_type = *p++;
    p++; /* flags */
    p++; /* flags2 */

    pos->satellites = *p++;
    p += 3; /* reserved */

    /* Longitude (4 bytes, 1e-7 deg) */
    pos->longitude = (int32_t)((p[0] << 0) | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
    p += 4;

    /* Latitude (4 bytes, 1e-7 deg) */
    pos->latitude = (int32_t)((p[0] << 0) | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
    p += 4;

    /* Height (4 bytes, mm) */
    p += 4; /* height (ellipsoid) */

    /* hMSL (4 bytes, mm) */
    pos->altitude_mm = (int32_t)((p[0] << 0) | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
    p += 4;

    /* Skip: hAcc(4), vAcc(4), velN(4), velE(4), velD(4), gspeed(4), headMot(4), spdAcc(4), headAcc(4) */
    p += 36;

    /* HDOP (2 bytes, 1e-2) */
    pos->hdop = (uint16_t)((p[0] << 0) | (p[1] << 8));

    pos->valid = (pos->fix_type >= 3);

    return 0;
}

static void m10_acquire_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct m10_data *data = p1;
    const struct gnss_u_blox_m10_i2c_config *cfg = &m10_i2c_config;
    uint8_t reg;
    int ret;

    LOG_INF("M10 acquisition thread started (10Hz)");

    while (1) {
        k_sleep(K_MSEC(cfg->fix_rate_ms));

        /* Check available bytes */
        reg = M10_REG_BYTES_AVAIL_H;
        ret = i2c_write_dt(&cfg->i2c, &reg, 1);
        if (ret < 0) {
            continue;
        }

        uint8_t avail_buf[2];
        ret = i2c_read_dt(&cfg->i2c, avail_buf, sizeof(avail_buf));
        if (ret < 0) {
            continue;
        }

        uint16_t avail = (avail_buf[0] << 8) | avail_buf[1];
        if (avail == 0) {
            continue;
        }

        if (avail > sizeof(data->parse_buf)) {
            avail = sizeof(data->parse_buf);
        }

        /* Read data */
        reg = M10_REG_DATA;
        ret = i2c_write_dt(&cfg->i2c, &reg, 1);
        if (ret < 0) {
            continue;
        }

        ret = i2c_read_dt(&cfg->i2c, data->parse_buf, avail);
        if (ret < 0) {
            continue;
        }

        /* Parse UBX messages */
        for (size_t i = 0; i < ret; i++) {
            uint8_t c = data->parse_buf[i];

            if (c == UBX_SYNC_1 && (i + 1) < ret && data->parse_buf[i + 1] == UBX_SYNC_2) {
                data->parse_len = 0;
                data->awaiting_sync = true;
            }

            if (data->awaiting_sync && data->parse_len < sizeof(data->parse_buf)) {
                data->parse_buf[data->parse_len++] = c;

                /* Full frame received? (sync + class + id + len + payload + checksum) */
                if (data->parse_len >= 8) {
                    uint16_t payload_len = data->parse_buf[6] | (data->parse_buf[7] << 8);
                    size_t frame_len = 8 + payload_len + 2;

                    if (data->parse_len >= frame_len) {
                        /* Check if it's NAV-PVT */
                        if (data->parse_len >= 10 &&
                            data->parse_buf[2] == UBX_CLASS_NAV &&
                            data->parse_buf[3] == UBX_MSG_NAV_PVT) {

                            struct gps_position pos;
                            memset(&pos, 0, sizeof(pos));
                            pos.cpu_timestamp_us = k_ticks_to_us_ceil32(k_cycle_get_32());

                            if (m10_parse_ubx_nav_pvt(&pos, data->parse_buf + 6, payload_len) == 0) {
                                if (pos.valid) {
                                    atomic_t write_idx = atomic_get(&data->write_idx);
                                    data->ring_buffer[write_idx] = pos;
                                    atomic_inc(&data->write_idx);
                                    if (atomic_get(&data->write_idx) >= RING_BUFFER_SIZE) {
                                        atomic_set(&data->write_idx, 0);
                                    }
                                }
                            }
                        }

                        data->parse_len = 0;
                        data->awaiting_sync = false;
                    }
                }
            }
        }
    }
}

K_THREAD_DEFINE(m10_acquire_tid, 1024, m10_acquire_thread, &m10_data_instance, NULL, NULL, 3, 0, 0);

int gps_get_latest(struct gps_position *pos)
{
    struct m10_data *data = &m10_data_instance;
    atomic_t idx = atomic_get(&data->read_idx);
    *pos = data->ring_buffer[idx];
    return 0;
}

int gps_get_latest_if_fresh(struct gps_position *pos, uint32_t max_age_ms)
{
    struct m10_data *data = &m10_data_instance;
    struct gps_position tmp;
    atomic_t write_idx = atomic_get(&data->write_idx);
    atomic_t read_idx = atomic_get(&data->read_idx);

    if (write_idx == read_idx) {
        return -ENODATA;
    }

    tmp = data->ring_buffer[read_idx];
    if (!tmp.valid) {
        return -ENODATA;
    }

    uint32_t now_us = k_ticks_to_us_ceil32(k_cycle_get_32());
    uint32_t age_us = now_us - tmp.cpu_timestamp_us;
    if ((age_us / 1000) > max_age_ms) {
        return -ETIMEDOUT;
    }

    *pos = tmp;
    return 0;
}

uint8_t gps_get_satellites(void)
{
    struct gps_position pos;
    if (gps_get_latest(&pos) == 0 && pos.valid) {
        return pos.satellites;
    }
    return 0;
}

bool gps_has_fix(void)
{
    struct gps_position pos;
    if (gps_get_latest(&pos) == 0 && pos.valid && pos.fix_type >= 3) {
        return true;
    }
    return false;
}

static int m10_init(const struct device *dev)
{
    struct m10_data *data = dev->data;
    int ret;

    if (!device_is_ready(m10_i2c_config.i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    data->parse_len = 0;
    data->awaiting_sync = false;
    atomic_set(&data->write_idx, 0);
    atomic_set(&data->read_idx, 0);

    ret = m10_configure(dev);
    if (ret < 0) {
        LOG_ERR("Failed to configure M10: %d", ret);
    }

    LOG_INF("M10 GNSS driver initialized on I2C addr 0x%02x",
            m10_i2c_config.i2c.addr);

    return 0;
}

static DEVICE_API(gnss, gnss_api) = {
};

static const struct gnss_u_blox_m10_config {
    uint32_t dummy;
} m10_cfg = { .dummy = 0 };

DEVICE_DEFINE(m10_gnss, "M10_GNSS",
            m10_init,
            NULL,
            &m10_data_instance,
            &m10_cfg,
            POST_KERNEL, CONFIG_GNSS_INIT_PRIORITY,
            &gnss_api);