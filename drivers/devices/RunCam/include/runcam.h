/*
 * Protocol reference:
 *   https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
 *
 * Packet format (host -> camera):
 *   [0] 0xCC           Header
 *   [1] <command>      Command ID
 *   [2] <action>       Action / parameter  (omitted for GET_DEVICE_INFO)
 *   [n] <crc8>         CRC-8/DVB-S2 over bytes [0..n-1]
 *
 * Packet format (camera -> host):
 *   [0] 0xCC           Header
 *   [1..n-1] <data>    Response payload
 *   [n] <crc8>         CRC-8/DVB-S2 over bytes [0..n-1]
 *
 * CRC polynomial: 0xD5  (DVB-S2 / MPEG-2 variant used by RunCam)
 */

#ifndef RUNCAM_H
#define RUNCAM_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Protocol constants ──────────────────────────────────────────────────── */

#define RUNCAM_HEADER           0xCCU

/* Command IDs */
#define RUNCAM_CMD_GET_DEVICE_INFO      0x00U
#define RUNCAM_CMD_CAMERA_CONTROL       0x01U
#define RUNCAM_CMD_5KEY_SIMULATION_PRESS    0x02U
#define RUNCAM_CMD_5KEY_SIMULATION_RELEASE  0x03U
#define RUNCAM_CMD_5KEY_CONNECTION      0x04U

/* Camera control action codes (used with RUNCAM_CMD_CAMERA_CONTROL) */
#define RUNCAM_ACTION_WIFI_BTN          0x01U   /* short: WiFi/confirm  */
#define RUNCAM_ACTION_POWER_BTN         0x02U   /* short: Power/next    */
#define RUNCAM_ACTION_CHANGE_MODE       0x03U   /* change mode          */
#define RUNCAM_ACTION_START_RECORDING   0x05U   /* dedicated start      */
#define RUNCAM_ACTION_STOP_RECORDING    0x06U   /* dedicated stop       */

/* Feature flags returned by GET_DEVICE_INFO */
#define RUNCAM_FEATURE_SIMULATE_POWER_BUTTON    (1U << 0)
#define RUNCAM_FEATURE_SIMULATE_WIFI_BUTTON     (1U << 1)
#define RUNCAM_FEATURE_CHANGE_MODE              (1U << 2)
#define RUNCAM_FEATURE_5KEY_OSD                 (1U << 3)
#define RUNCAM_FEATURE_SETTINGS_ACCESS          (1U << 4)
#define RUNCAM_FEATURE_DISPLAYPORT              (1U << 5)
#define RUNCAM_FEATURE_START_RECORDING          (1U << 6)
#define RUNCAM_FEATURE_STOP_RECORDING           (1U << 7)

/* Packet size limits */
#define RUNCAM_MAX_PACKET_LEN   16U
#define RUNCAM_RESP_DEVINFO_LEN  5U   /* header + proto_ver + feat_lo + feat_hi + crc */
#define RUNCAM_RESP_CTRL_LEN     2U   /* header + crc  (camera control has no payload) */

/* Timeouts */
#define RUNCAM_RESPONSE_TIMEOUT_MS   500
#define RUNCAM_BOOT_DELAY_MS        3000

/* ── Recording status ────────────────────────────────────────────────────── */

typedef enum {
    RUNCAM_STATUS_UNKNOWN = 0,
    RUNCAM_STATUS_IDLE,
    RUNCAM_STATUS_RECORDING,
} runcam_recording_status_t;

/* ── Device information ──────────────────────────────────────────────────── */

typedef struct {
    uint8_t  protocol_version;
    uint16_t features;          /* bitmask of RUNCAM_FEATURE_* */
    bool     initialized;
} runcam_device_info_t;

/* ── Driver context (opaque to caller) ───────────────────────────────────── */

typedef struct {
    const struct device        *uart_dev;
    runcam_device_info_t        info;
    runcam_recording_status_t   recording_status;

    /* RX ring buffer for ISR-driven reception */
    struct k_sem    rx_sem;
    uint8_t         rx_buf[RUNCAM_MAX_PACKET_LEN];
    uint8_t         rx_len;
} runcam_ctx_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief Initialise the RunCam driver.
 *
 * @param ctx       Caller-allocated context structure.
 * @param uart_dev  Zephyr UART device (e.g. DEVICE_DT_GET(DT_ALIAS(runcam_uart))).
 * @return 0 on success, negative errno on failure.
 */
int runcam_init(runcam_ctx_t *ctx, const struct device *uart_dev);

/**
 * @brief Query the camera for device info and supported features.
 *        Blocks until a response is received or timeout expires.
 *
 * @param ctx  Driver context.
 * @return 0 on success, -ETIMEDOUT if no response, -EIO on CRC error.
 */
int runcam_get_device_info(runcam_ctx_t *ctx);

/**
 * @brief Send a camera control command.
 *
 * @param ctx     Driver context.
 * @param action  One of the RUNCAM_ACTION_* constants.
 * @return 0 on success, negative errno on failure.
 */
int runcam_send_control(runcam_ctx_t *ctx, uint8_t action);

/**
 * @brief Start recording.
 *        Automatically chooses the correct action code based on device features.
 *
 * @param ctx  Driver context.
 * @return 0 on success, negative errno on failure.
 */
int runcam_start_recording(runcam_ctx_t *ctx);

/**
 * @brief Stop recording.
 *        Automatically chooses the correct action code based on device features.
 *
 * @param ctx  Driver context.
 * @return 0 on success, negative errno on failure.
 */
int runcam_stop_recording(runcam_ctx_t *ctx);

/**
 * @brief Get the current (locally-tracked) recording status.
 *
 * @param ctx  Driver context.
 * @return RUNCAM_STATUS_RECORDING, RUNCAM_STATUS_IDLE, or RUNCAM_STATUS_UNKNOWN.
 */
runcam_recording_status_t runcam_get_recording_status(const runcam_ctx_t *ctx);

/**
 * @brief Return a human-readable string for a recording status.
 */
const char *runcam_status_str(runcam_recording_status_t status);

/* ── CRC helper (exposed for unit testing) ───────────────────────────────── */

/**
 * @brief Compute CRC-8/DVB-S2 used by the RunCam protocol.
 *
 * @param data  Input bytes.
 * @param len   Number of bytes.
 * @return 8-bit CRC.
 */
uint8_t runcam_crc8(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* RUNCAM_H */