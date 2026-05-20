#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/modem/ubx/protocol.h>

#include "gnss_u_blox_m10_i2c.h"
#include "gnss_u_blox_m10.h"

LOG_MODULE_REGISTER(gnss_u_blox_m10, CONFIG_GNSS_LOG_LEVEL);

#define M10_GNSS_NODE DT_NODELABEL(gps)

#define RAW_FRAME_MAX_SIZE 256
#define PARSE_FRESHNESS_THRESHOLD_MS 300

struct m10_data {
    struct k_mutex lock;

    /* Parsed result (read by getters, written by parser) */
    struct gps_position parsed_data;
    uint32_t parsed_timestamp_us;
    bool parsed_data_ready;

    /* Latest complete raw UBX frame (written by acquire thread, read by parser) */
    uint8_t latest_raw_frame[RAW_FRAME_MAX_SIZE];
    size_t latest_raw_frame_len;
    bool raw_frame_ready;

    /* Parse state machine (owned by acquire thread) */
    uint8_t parse_buf[RAW_FRAME_MAX_SIZE];
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
    uint8_t cfg_msg[12];

    cfg_msg[0] = 0xB5;
    cfg_msg[1] = 0x62;
    cfg_msg[2] = 0x06;
    cfg_msg[3] = 0x01;
    cfg_msg[4] = 3;  /* payload length */
    cfg_msg[5] = 0;
    cfg_msg[6] = msg_class;
    cfg_msg[7] = msg_id;
    cfg_msg[8] = rate;  /* rate for current destination (I2C) */

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 9; i++) {
        ck_a += cfg_msg[i];
        ck_b += ck_a;
    }
    cfg_msg[9] = ck_a;
    cfg_msg[10] = ck_b;
    cfg_msg[11] = 0;

    LOG_DBG("CFG-MSG: class=0x%02X, id=0x%02X, rate=%d", msg_class, msg_id, rate);

    return i2c_write_dt(i2c, cfg_msg, 11);
}

static int m10_send_cfg_valset(const struct i2c_dt_spec *i2c, uint32_t key_id, uint32_t value)
{
    uint8_t cfg_msg[18];
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

    cfg_msg[0] = 0xB5;
    cfg_msg[1] = 0x62;
    cfg_msg[2] = 0x06;
    cfg_msg[3] = 0x8A;
    cfg_msg[4] = sizeof(payload) & 0xFF;
    cfg_msg[5] = (sizeof(payload) >> 8) & 0xFF;

    memcpy(&cfg_msg[6], &payload, sizeof(payload));

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6 + (int)sizeof(payload); i++) {
        ck_a += cfg_msg[i];
        ck_b += ck_a;
    }
    cfg_msg[6 + sizeof(payload)] = ck_a;
    cfg_msg[6 + sizeof(payload) + 1] = ck_b;

    LOG_DBG("CFG-VALSET: key=0x%08X, value=%u", key_id, value);

    return i2c_write_dt(i2c, cfg_msg, 8 + sizeof(payload));
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
        uint8_t rate_msg[14];
        rate_msg[0] = 0xB5;
        rate_msg[1] = 0x62;
        rate_msg[2] = 0x06;
        rate_msg[3] = 0x08;
        rate_msg[4] = 6;  /* payload length */
        rate_msg[5] = 0;
        rate_msg[6] = 40 & 0xFF;
        rate_msg[7] = (40 >> 8) & 0xFF;
        rate_msg[8] = 40 & 0xFF;
        rate_msg[9] = (40 >> 8) & 0xFF;
        rate_msg[10] = 40 & 0xFF;
        rate_msg[11] = (40 >> 8) & 0xFF;

        uint8_t ck_a = 0, ck_b = 0;
        for (int i = 2; i < 12; i++) {
            ck_a += rate_msg[i];
            ck_b += ck_a;
        }
        rate_msg[12] = ck_a;
        rate_msg[13] = ck_b;

        i2c_write_dt(i2c, rate_msg, 14);
    }
    k_sleep(K_MSEC(50));

    /* Enable UBX-NAV-PVT on I2C using CFG-MSG (class=NAV, id=PVT, rate=1) */
    ret = m10_send_cfg_msg(i2c, 0x01, 0x07, 1);
    if (ret < 0) {
        LOG_WRN("Failed to enable NAV-PVT: %d", ret);
    }
    k_sleep(K_MSEC(50));

    /* Disable NMEA GGA on I2C (class=0xF0, id=0x00) */
    ret = m10_send_cfg_msg(i2c, 0xF0, 0x00, 0);
    if (ret < 0) {
        LOG_WRN("Failed to disable NMEA GGA: %d", ret);
    }
    k_sleep(K_MSEC(50));

    /* Disable NMEA GLL on I2C (class=0xF0, id=0x01) */
    ret = m10_send_cfg_msg(i2c, 0xF0, 0x01, 0);
    k_sleep(K_MSEC(50));

    /* Disable NMEA GSA on I2C (class=0xF0, id=0x02) */
    ret = m10_send_cfg_msg(i2c, 0xF0, 0x02, 0);
    k_sleep(K_MSEC(50));

    /* Disable NMEA RMC on I2C (class=0xF0, id=0x04) */
    ret = m10_send_cfg_msg(i2c, 0xF0, 0x04, 0);
    k_sleep(K_MSEC(50));
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

        /* Read data */
        reg = M10_REG_DATA;
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

        /* Run state machine to find complete UBX frames */
        for (size_t i = 0; i < ret; i++) {
            uint8_t c = data->parse_buf[i];

            if (c == UBX_PREAMBLE_SYNC_CHAR_1 && (i + 1) < ret && data->parse_buf[i + 1] == UBX_PREAMBLE_SYNC_CHAR_2) {
                data->parse_len = 0;
                data->awaiting_sync = true;
            }

            if (data->awaiting_sync && data->parse_len < sizeof(data->parse_buf)) {
                data->parse_buf[data->parse_len++] = c;

                if (data->parse_len >= 6) {
                    uint16_t payload_len = data->parse_buf[4] | (data->parse_buf[5] << 8);
                    size_t frame_len = 6 + payload_len + 2;

                    if (payload_len > 512) {
                        data->awaiting_sync = false;
                        data->parse_len = 0;
                        continue;
                    }

                    if (data->parse_len >= frame_len) {
                        /* Complete frame found - copy to latest_raw_frame under lock */
                        k_mutex_lock(&data->lock, K_FOREVER);
                        if (frame_len <= RAW_FRAME_MAX_SIZE) {
                            memcpy(data->latest_raw_frame, data->parse_buf, frame_len);
                            data->latest_raw_frame_len = frame_len;
                            data->raw_frame_ready = true;
                        }
                        k_mutex_unlock(&data->lock);

                        data->awaiting_sync = false;
                        data->parse_len = 0;
                    }
                }
            }
        }
    }
}

K_THREAD_DEFINE(m10_acquire_tid, 1024, m10_acquire_thread, &m10_data_instance, NULL, NULL, 3, 0, 0);

/**
 * @brief Parse the latest raw UBX frame and update parsed_data
 *
 * Must be called with data->lock held.
 *
 * @return 0 on success, -ENODATA if no raw frame available, -EINVAL if parse failed
 */
static int m10_do_parse(struct m10_data *data)
{
    if (!data->raw_frame_ready) {
        return -ENODATA;
    }

    if (data->latest_raw_frame_len < 6) {
        return -EINVAL;
    }

    uint16_t payload_len = data->latest_raw_frame[4] | (data->latest_raw_frame[5] << 8);

    if (data->latest_raw_frame[2] != UBX_CLASS_ID_NAV ||
        data->latest_raw_frame[3] != UBX_MSG_ID_NAV_PVT) {
        return -EINVAL;
    }

    struct gps_position pos;
    memset(&pos, 0, sizeof(pos));
    pos.cpu_timestamp_us = k_ticks_to_us_ceil32(k_cycle_get_32());

    int ret = m10_parse_ubx_nav_pvt(&pos, data->latest_raw_frame + 6, payload_len);
    if (ret != 0 || !pos.valid) {
        return -EINVAL;
    }

    LOG_DBG_RATELIMIT_RATE(1000, "GPS: %02d/%02d/%04d %02d:%02d:%02d.%03u | "
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

    data->parsed_data = pos;
    data->parsed_timestamp_us = pos.cpu_timestamp_us;
    data->parsed_data_ready = true;
    data->raw_frame_ready = false;

    return 0;
}

/**
 * @brief Parse latest raw data if parsed data is stale
 *
 * If parsed data is older than PARSE_FRESHNESS_THRESHOLD_MS, attempts to parse
 * the latest raw frame. Always returns the best available parsed data.
 *
 * @param pos Pointer to position structure to fill
 * @return 0 on success, -ENODATA if no valid data available
 */
static int gps_parse_latest(struct gps_position *pos)
{
    struct m10_data *data = &m10_data_instance;

    k_mutex_lock(&data->lock, K_FOREVER);

    bool need_parse = false;
    if (data->parsed_data_ready) {
        uint32_t now_us = k_ticks_to_us_ceil32(k_cycle_get_32());
        uint32_t age_us = now_us - data->parsed_timestamp_us;
        if ((age_us / 1000) > PARSE_FRESHNESS_THRESHOLD_MS) {
            need_parse = true;
        }
    } else {
        need_parse = true;
    }

    if (need_parse && data->raw_frame_ready) {
        m10_do_parse(data);
    }

    if (!data->parsed_data_ready) {
        k_mutex_unlock(&data->lock);
        return -ENODATA;
    }

    *pos = data->parsed_data;
    k_mutex_unlock(&data->lock);

    return 0;
}

int gps_get_latest(struct gps_position *pos)
{
    return gps_parse_latest(pos);
}

int gps_get_latest_if_fresh(struct gps_position *pos, uint32_t max_age_ms)
{
    struct m10_data *data = &m10_data_instance;

    k_mutex_lock(&data->lock, K_FOREVER);

    if (!data->parsed_data_ready) {
        k_mutex_unlock(&data->lock);
        return -ENODATA;
    }

    uint32_t now_us = k_ticks_to_us_ceil32(k_cycle_get_32());
    uint32_t age_us = now_us - data->parsed_timestamp_us;
    if ((age_us / 1000) > max_age_ms) {
        k_mutex_unlock(&data->lock);
        return -ETIMEDOUT;
    }

    *pos = data->parsed_data;
    k_mutex_unlock(&data->lock);

    return 0;
}

uint8_t gps_get_satellites(void)
{
    struct m10_data *data = &m10_data_instance;

    k_mutex_lock(&data->lock, K_FOREVER);
    uint8_t sats = (data->parsed_data_ready && data->parsed_data.valid)
                   ? data->parsed_data.satellites : 0;
    k_mutex_unlock(&data->lock);

    return sats;
}

bool gps_has_fix(void)
{
    struct m10_data *data = &m10_data_instance;

    k_mutex_lock(&data->lock, K_FOREVER);
    bool fix = data->parsed_data_ready && data->parsed_data.valid && data->parsed_data.fix_type >= 3;
    k_mutex_unlock(&data->lock);

    return fix;
}

int32_t gps_get_latitude(void)
{
    struct m10_data *data = &m10_data_instance;

    k_mutex_lock(&data->lock, K_FOREVER);
    int32_t lat = (data->parsed_data_ready && data->parsed_data.valid)
                  ? data->parsed_data.latitude : 0;
    k_mutex_unlock(&data->lock);

    return lat;
}

int32_t gps_get_longitude(void)
{
    struct m10_data *data = &m10_data_instance;

    k_mutex_lock(&data->lock, K_FOREVER);
    int32_t lon = (data->parsed_data_ready && data->parsed_data.valid)
                  ? data->parsed_data.longitude : 0;
    k_mutex_unlock(&data->lock);

    return lon;
}

int32_t gps_get_altitude(void)
{
    struct m10_data *data = &m10_data_instance;

    k_mutex_lock(&data->lock, K_FOREVER);
    int32_t alt = (data->parsed_data_ready && data->parsed_data.valid)
                  ? data->parsed_data.altitude_mm : 0;
    k_mutex_unlock(&data->lock);

    return alt;
}

static int m10_init(const struct device *dev)
{
    struct m10_data *data = dev->data;
    int ret;

    if (!device_is_ready(m10_i2c_config.i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    k_mutex_init(&data->lock);

    data->parse_len = 0;
    data->awaiting_sync = false;
    data->raw_frame_ready = false;
    data->latest_raw_frame_len = 0;
    data->parsed_data_ready = false;
    data->parsed_timestamp_us = 0;
    memset(&data->parsed_data, 0, sizeof(data->parsed_data));
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