/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * u-blox M10 GNSS driver over I2C with non-blocking API
 * Uses Zephyr's UBX parsing library
 */

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/modem/ubx/protocol.h>

#include "gnss_u_blox_m10_i2c.h"
#include "gnss_u_blox_m10.h"

LOG_MODULE_REGISTER(gnss_u_blox_m10, CONFIG_GNSS_LOG_LEVEL);

#define M10_GNSS_NODE DT_NODELABEL(gps)

#define RING_BUFFER_SIZE 2

struct m10_data {
    struct gps_position ring_buffer[RING_BUFFER_SIZE];
    atomic_t write_idx;
    atomic_t read_idx;
    uint8_t parse_buf[256];
    size_t parse_len;
    bool awaiting_sync;
    bool configured;
};

static struct m10_data m10_data_instance;

static const struct gnss_u_blox_m10_i2c_config m10_i2c_config = {
    .i2c = I2C_DT_SPEC_GET(M10_GNSS_NODE),
    .fix_rate_ms = 40,
};

static int m10_wakeup(const struct i2c_dt_spec *i2c)
{
    uint8_t poll_msg[16];
    int frame_len;

    frame_len = ubx_frame_encode(UBX_CLASS_ID_MON, UBX_MSG_ID_MON_VER,
                                  NULL, 0, poll_msg, sizeof(poll_msg));
    if (frame_len < 0) {
        LOG_ERR("Failed to encode MON-VER poll");
        return frame_len;
    }

    int ret = i2c_write_dt(i2c, poll_msg, frame_len);
    if (ret < 0) {
        LOG_WRN("M10 wake-up poll failed: %d", ret);
    }
    k_sleep(K_MSEC(50));
    return ret;
}

static int m10_send_cfg_msg(const struct i2c_dt_spec *i2c, uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    uint8_t cfg_msg[16];
    int frame_len;

    struct {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rate;
    } __packed payload = {
        .msg_class = msg_class,
        .msg_id = msg_id,
        .rate = rate
    };

    frame_len = ubx_frame_encode(UBX_CLASS_ID_CFG, UBX_MSG_ID_CFG_MSG,
                                  (uint8_t *)&payload, sizeof(payload),
                                  cfg_msg, sizeof(cfg_msg));
    if (frame_len < 0) {
        LOG_ERR("Failed to encode CFG-MSG frame");
        return frame_len;
    }

    LOG_DBG("CFG-MSG: class=0x%02X, id=0x%02X, rate=%d", msg_class, msg_id, rate);

    return i2c_write_dt(i2c, cfg_msg, frame_len);
}

static int m10_send_cfg_valset(const struct i2c_dt_spec *i2c, uint32_t key_id, uint32_t value)
{
    uint8_t cfg_msg[32];
    int frame_len;

    struct {
        uint8_t ver;
        uint8_t layer;
        uint16_t reserved;
        uint32_t key;
        uint32_t value;
    } __packed payload = {
        .ver = 0x00,
        .layer = 0x00,
        .reserved = 0x0000,
        .key = key_id,
        .value = value
    };

    frame_len = ubx_frame_encode(UBX_CLASS_ID_CFG, UBX_MSG_ID_CFG_VAL_SET,
                                  (uint8_t *)&payload, sizeof(payload),
                                  cfg_msg, sizeof(cfg_msg));
    if (frame_len < 0) {
        LOG_ERR("Failed to encode CFG-VALSET frame");
        return frame_len;
    }

    LOG_DBG("CFG-VALSET: key=0x%08X, value=%u", key_id, value);

    return i2c_write_dt(i2c, cfg_msg, frame_len);
}

static int m10_configure(const struct device *dev)
{
    ARG_UNUSED(dev);
    const struct i2c_dt_spec *i2c = &m10_i2c_config.i2c;
    int ret;

    LOG_DBG("Configuring M10 for 25Hz UBX-NAV-PVT on I2C");

    k_sleep(K_MSEC(2000));

    ret = m10_wakeup(i2c);
    if (ret < 0) {
        LOG_WRN("M10 wake-up failed, proceeding with config anyway...");
    }

    k_sleep(K_MSEC(100));

    /* Set measurement rate to 40ms (25Hz) using CFG-RATE */
    {
        uint8_t rate_msg[16];
        struct {
            uint16_t meas_rate;
            uint16_t nav_rate;
            uint16_t time_ref;
        } __packed payload = {
            .meas_rate = 40,
            .nav_rate = 40,
            .time_ref = 40
        };
        int len = ubx_frame_encode(UBX_CLASS_ID_CFG, UBX_MSG_ID_CFG_RATE,
                                    (uint8_t *)&payload, sizeof(payload),
                                    rate_msg, sizeof(rate_msg));
        if (len > 0) {
            i2c_write_dt(i2c, rate_msg, len);
        }
    }
    k_sleep(K_MSEC(50));

    /* Enable UBX-NAV-PVT on I2C (DDC) - CFG-MSGOUT key */
    ret = m10_send_cfg_valset(i2c, 0x20910007, 1);
    if (ret < 0) {
        LOG_WRN("Failed to enable NAV-PVT: %d", ret);
    }
    k_sleep(K_MSEC(50));

    /* Enable UBX protocol on I2C */
    ret = m10_send_cfg_valset(i2c, 0x10720001, 1);
    if (ret < 0) {
        LOG_WRN("Failed to enable UBX output: %d", ret);
    }
    k_sleep(K_MSEC(50));

    /* Disable NMEA protocol on I2C */
    ret = m10_send_cfg_valset(i2c, 0x10720002, 0);
    if (ret < 0) {
        LOG_WRN("Failed to disable NMEA: %d", ret);
    }
    k_sleep(K_MSEC(50));

    k_sleep(K_MSEC(10));

    LOG_INF("M10 configuration complete");
    return 0;
}

static int m10_parse_nmea_gga(struct gps_position *pos, const char *sentence)
{
    if (sentence[0] != '$' || sentence[3] != 'G' || sentence[4] != 'G' || sentence[5] != 'A') {
        return -EINVAL;
    }

    char *tokens[15];
    int count = 0;
    char buf[128];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    char *tok = strtok(buf, ",");
    while (tok && count < 15) {
        tokens[count++] = tok;
        tok = strtok(NULL, ",");
    }

    if (count < 10) {
        return -EINVAL;
    }

    int fix = atoi(tokens[6]);
    if (fix < 1) {
        return -EINVAL;
    }

    pos->fix_type = (fix >= 3) ? 3 : (fix >= 2 ? 2 : 1);
    pos->satellites = atoi(tokens[7]);
    pos->hdop = (uint16_t)(atof(tokens[8]) * 100);

    if (tokens[2] && tokens[3] && tokens[4] && tokens[5]) {
        double lat = atof(tokens[2]);
        double lon = atof(tokens[4]);

        int lat_deg = (int)(lat / 100);
        int lat_min = (int)lat % 100;
        double lat_sec = lat - lat_deg * 100;
        pos->latitude = (int32_t)((lat_deg + lat_min / 60.0 + lat_sec / 6000.0) * 1e7);
        if (tokens[3][0] == 'S') {
            pos->latitude = -pos->latitude;
        }

        int lon_deg = (int)(lon / 100);
        int lon_min = (int)lon % 100;
        double lon_sec = lon - lon_deg * 100;
        pos->longitude = (int32_t)((lon_deg + lon_min / 60.0 + lon_sec / 6000.0) * 1e7);
        if (tokens[5][0] == 'W') {
            pos->longitude = -pos->longitude;
        }
    }

    if (tokens[9]) {
        pos->altitude_mm = (int32_t)(atof(tokens[9]) * 1000);
    }

    pos->valid = true;
    return 0;
}

static int m10_parse_ubx_nav_pvt(struct gps_position *pos, const uint8_t *data, size_t len)
{
    if (len < sizeof(struct ubx_nav_pvt)) {
        return -EINVAL;
    }

    const struct ubx_nav_pvt *pvt = (const struct ubx_nav_pvt *)data;

    pos->fix_type = pvt->fix_type;
    pos->satellites = pvt->nav.num_sv;
    pos->longitude = pvt->nav.longitude;
    pos->latitude = pvt->nav.latitude;
    pos->altitude_mm = pvt->nav.hmsl;
    pos->hdop = pvt->nav.pdop;

    pos->year = pvt->time.year;
    pos->month = pvt->time.month;
    pos->day = pvt->time.day;
    pos->hour = pvt->time.hour;
    pos->minute = pvt->time.minute;
    pos->second = pvt->time.second;
    pos->nanosecond = pvt->time.nano;
    pos->time_valid = (pvt->time.valid & 0x03);

    pos->itow_ms = pvt->time.itow;
    if (pos->time_valid & 0x03) {
        int64_t nano = pvt->time.nano;
        if (nano < 0) {
            pos->gps_timestamp_ns = ((uint64_t)pvt->time.itow * 1000000ULL) + (uint64_t)nano;
        } else {
            pos->gps_timestamp_ns = ((uint64_t)pvt->time.itow * 1000000ULL) + (uint64_t)nano;
        }
    } else {
        pos->gps_timestamp_ns = 0;
    }

    pos->speed_mm_s = pvt->nav.ground_speed;
    pos->heading_1e5 = pvt->nav.head_motion;
    pos->horiz_acc_mm = pvt->nav.horiz_acc;
    pos->vert_acc_mm = pvt->nav.vert_acc;

    pos->valid = (pvt->fix_type >= 3) && (pvt->flags & UBX_NAV_PVT_FLAGS_GNSS_FIX_OK);

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
    uint8_t config_retry_count = 0;

    LOG_INF("M10 acquisition thread started (25Hz)");

    while (1) {
        if (!data->configured && config_retry_count < 5) {
            ret = m10_configure(NULL);
            if (ret == 0) {
                data->configured = true;
                // LOG_DBG("M10 configuration successful!");
            } else {
                config_retry_count++;
                LOG_WRN("M10 config attempt %d/5 failed", config_retry_count);
            }
        }

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

        /* Read data - sequential write register address then read */
        uint8_t reg = M10_REG_DATA;
        ret = i2c_write_dt(&cfg->i2c, &reg, 1);
        if (ret < 0) {
            LOG_DBG("M10: write reg failed: %d", ret);
            continue;
        }

        k_sleep(K_MSEC(1));

        ret = i2c_read_dt(&cfg->i2c, data->parse_buf, avail);
        if (ret < 0) {
            LOG_DBG("M10: read failed: %d", ret);
            continue;
        }
        if (ret == 0 && avail > 0) {
            ret = avail;
        }

        /* Parse UBX messages */
        bool found_ubx = false;

        for (size_t i = 0; i < ret; i++) {
            uint8_t c = data->parse_buf[i];

            if (c == UBX_PREAMBLE_SYNC_CHAR_1 && (i + 1) < ret && data->parse_buf[i + 1] == UBX_PREAMBLE_SYNC_CHAR_2) {
                data->parse_len = 0;
                data->awaiting_sync = true;
                found_ubx = true;
            }

            if (data->awaiting_sync && data->parse_len < sizeof(data->parse_buf)) {
                data->parse_buf[data->parse_len++] = c;

                /* Full frame received? (sync + class + id + len + payload + checksum) */
                if (data->parse_len >= 6) {
                    uint16_t payload_len = data->parse_buf[4] | (data->parse_buf[5] << 8);
                    size_t frame_len = 6 + payload_len + 2;

                    if (payload_len > 512) {
                        data->awaiting_sync = false;
                        data->parse_len = 0;
                        continue;
                    }

                    if (data->parse_len >= frame_len) {
                        if (data->parse_buf[2] == UBX_CLASS_ID_NAV &&
                            data->parse_buf[3] == UBX_MSG_ID_NAV_PVT) {

                            struct gps_position pos;
                            memset(&pos, 0, sizeof(pos));
                            pos.cpu_timestamp_us = k_ticks_to_us_ceil32(k_cycle_get_32());

                            int parse_ret = m10_parse_ubx_nav_pvt(&pos, data->parse_buf + 6, payload_len);
                            if (parse_ret == 0 && pos.valid) {
                                LOG_DBG_RATELIMIT_RATE(1000,"GPS: %02d/%02d/%04d %02d:%02d:%02d.%03u | "
                                        "lat=%d, lon=%d, alt=%dmm | "
                                        "sats=%d, fix=%d, hdop=%d | "
                                        "speed=%dmm/s, heading=%d.%05d | "
                                        "acc: horiz=%umm, vert=%umm | "
                                        "gps_ns=%llu, cpu_us=%u",
                                        pos.day, pos.month, pos.year,
                                        pos.hour, pos.minute, pos.second, pos.nanosecond / 1000000,
                                        pos.latitude, pos.longitude, pos.altitude_mm,
                                        pos.satellites, pos.fix_type, pos.hdop,
                                        pos.speed_mm_s, pos.heading_1e5 / 100000, pos.heading_1e5 % 100000,
                                        pos.horiz_acc_mm, pos.vert_acc_mm,
                                        pos.gps_timestamp_ns, pos.cpu_timestamp_us);
                                atomic_t write_idx = atomic_get(&data->write_idx);
                                data->ring_buffer[write_idx] = pos;
                                atomic_inc(&data->write_idx);
                                if (atomic_get(&data->write_idx) >= RING_BUFFER_SIZE) {
                                    atomic_set(&data->write_idx, 0);
                                }
                            }
                        }
                        data->awaiting_sync = false;
                        data->parse_len = 0;
                    }
                }
            }
        }

        if (ret > 10) {
            for (size_t i = 0; i < ret - 5; i++) {
                if (data->parse_buf[i] == '$' && i + 80 < ret) {
                    size_t nmea_end = i;
                    for (size_t j = i + 4; j < ret && j < i + 100; j++) {
                        if (data->parse_buf[j] == '\n' || data->parse_buf[j] == '\r') {
                            nmea_end = j;
                            break;
                        }
                    }
                    if (nmea_end > i + 10 && nmea_end < i + 100) {
                        char nmea_str[100];
                        for (size_t k = 0; k < nmea_end - i && k < sizeof(nmea_str)-1; k++) {
                            nmea_str[k] = data->parse_buf[i + k];
                        }
                        nmea_str[nmea_end - i] = 0;

                        struct gps_position pos;
                        memset(&pos, 0, sizeof(pos));
                        pos.cpu_timestamp_us = k_ticks_to_us_ceil32(k_cycle_get_32());

                        if (m10_parse_nmea_gga(&pos, nmea_str) == 0 && pos.valid) {
                            LOG_DBG_RATELIMIT("GPS NMEA: lat=%d, lon=%d, alt=%dmm, sats=%d, fix=%d",
                                    pos.latitude, pos.longitude, pos.altitude_mm,
                                    pos.satellites, pos.fix_type);
                            atomic_t write_idx = atomic_get(&data->write_idx);
                            data->ring_buffer[write_idx] = pos;
                            atomic_inc(&data->write_idx);
                            if (atomic_get(&data->write_idx) >= RING_BUFFER_SIZE) {
                                atomic_set(&data->write_idx, 0);
                            }
                            break;
                        }
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
    data->configured = false;

    ret = m10_configure(dev);
    if (ret < 0) {
        LOG_WRN("M10 not configured yet, will retry in acquire thread");
    } else {
        data->configured = true;
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