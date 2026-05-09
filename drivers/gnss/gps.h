/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public GPS API for M10 GNSS driver
 */

#ifndef _PANORAMIX_GPS_H_
#define _PANORAMIX_GPS_H_

#include <stdint.h>
#include <stdbool.h>

struct gps_position {
    uint32_t cpu_timestamp_us;
    int32_t latitude;      /* 1e-7 degrees */
    int32_t longitude;     /* 1e-7 degrees */
    int32_t altitude_mm;   /* mm above mean sea level */
    uint8_t fix_type;     /* 0=none, 2=2D, 3=3D */
    uint8_t satellites;    /* number of satellites used */
    uint16_t hdop;         /* horizontal DOP * 100 */
    bool valid;
};

/**
 * @brief Get the latest GPS position
 *
 * Non-blocking - just reads from ring buffer
 *
 * @param pos Pointer to position structure to fill
 * @return 0 on success, -ENODATA if no valid data
 */
int gps_get_latest(struct gps_position *pos);

/**
 * @brief Get latest position if it's fresh enough
 *
 * @param pos Pointer to position structure to fill
 * @param max_age_ms Maximum age in milliseconds
 * @return 0 on success, -ENODATA if no valid data, -ETIMEDOUT if data too old
 */
int gps_get_latest_if_fresh(struct gps_position *pos, uint32_t max_age_ms);

/**
 * @brief Get number of satellites used in fix
 *
 * @return Number of satellites (0 if no fix)
 */
uint8_t gps_get_satellites(void);

/**
 * @brief Check if we have a 3D fix
 *
 * @return true if we have a valid 3D fix
 */
bool gps_has_fix(void);

/**
 * @brief Get latitude in degrees (scaled by 1e-7)
 *
 * @return Latitude in 1e-7 degrees
 */
static inline int32_t gps_get_latitude(void)
{
    struct gps_position pos;
    if (gps_get_latest(&pos) == 0 && pos.valid) {
        return pos.latitude;
    }
    return 0;
}

/**
 * @brief Get longitude in degrees (scaled by 1e-7)
 *
 * @return Longitude in 1e-7 degrees
 */
static inline int32_t gps_get_longitude(void)
{
    struct gps_position pos;
    if (gps_get_latest(&pos) == 0 && pos.valid) {
        return pos.longitude;
    }
    return 0;
}

/**
 * @brief Get altitude in mm above mean sea level
 *
 * @return Altitude in mm
 */
static inline int32_t gps_get_altitude(void)
{
    struct gps_position pos;
    if (gps_get_latest(&pos) == 0 && pos.valid) {
        return pos.altitude_mm;
    }
    return 0;
}

#endif /* _PANORAMIX_GPS_H_ */