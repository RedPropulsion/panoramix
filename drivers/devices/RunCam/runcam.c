/*
 * runcam.c  –  RunCam Device Protocol v1 driver for Zephyr
 *
 * Implements packet building, CRC-8/DVB-S2, ISR-driven UART receive,
 * and the three user-facing operations:
 *   - GET_DEVICE_INFO   -> query features / protocol version
 *   - START_RECORDING   -> RCDEVICE_PROTOCOL_CHANGE_START_RECORDING (0x05)
 *   - STOP_RECORDING    -> RCDEVICE_PROTOCOL_CHANGE_STOP_RECORDING  (0x06)
 *
 * For cameras that do NOT advertise FEATURE_START/STOP_RECORDING the
 * driver falls back to a simulated power-button press (action 0x02)
 * which toggles recording on most RunCam Split models.
 */

#include "runcam.h"
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(runcam, LOG_LEVEL_DBG);

/* ── CRC-8/DVB-S2 ────────────────────────────────────────────────────────── */
/*
 * Polynomial: x^8 + x^7 + x^3 + x^2 + x + 1  (0xD5, reflected: 0xAB)
 * This is the same variant used by Betaflight / ArduPilot for the RunCam
 * protocol and verified against the reference implementation.
 */
uint8_t runcam_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00U;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0xD5U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ── Internal helpers ────────────────────────────────────────────────────── */

/**
 * Build and transmit a RunCam packet over UART.
 *
 * Packet layout:
 *   [0]          0xCC  header
 *   [1]          command
 *   [2]          action (only when have_action == true)
 *   [last byte]  CRC-8/DVB-S2 of all preceding bytes
 */
static int send_packet(runcam_ctx_t *ctx, uint8_t command, bool have_action, uint8_t action)
{
    uint8_t buf[4];
    uint8_t len;

    buf[0] = RUNCAM_HEADER;
    buf[1] = command;

    if (have_action) {
        buf[2] = action;
        len = 4U;
    } else {
        len = 3U;
    }

    /* CRC covers all bytes except the CRC byte itself */
    buf[len - 1U] = runcam_crc8(buf, len - 1U);

    LOG_HEXDUMP_DBG(buf, len, "TX");

    for (uint8_t i = 0; i < len; i++) {
        uart_poll_out(ctx->uart_dev, buf[i]);
    }

    return 0;
}

/**
 * ISR callback – accumulate bytes into rx_buf, signal when a complete
 * packet has arrived.
 *
 * The RunCam protocol always starts with 0xCC.  We use that to re-sync
 * if the buffer gets out of phase.
 */
static void uart_isr_cb(const struct device *dev, void *user_data) {
    runcam_ctx_t *ctx = (runcam_ctx_t *)user_data;

    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uint8_t byte;

        if (uart_fifo_read(dev, &byte, 1) != 1) {
            break;
        }

        /* Re-sync: if buffer is empty we must see the header first */
        if (ctx->rx_len == 0U && byte != RUNCAM_HEADER) {
            LOG_WRN("dropped byte 0x%02X (no header)", byte);
            continue;
        }

        if (ctx->rx_len < RUNCAM_MAX_PACKET_LEN) {
            ctx->rx_buf[ctx->rx_len++] = byte;
        }

        /*
         * Signal the waiting thread.  We signal on every byte so the
         * receiver loop can check packet completeness incrementally.
         * The semaphore count acts as a byte-available counter.
         */
        k_sem_give(&ctx->rx_sem);
    }
}

/**
 * Wait for exactly `expected_len` bytes to arrive, then validate CRC.
 *
 * @return  0 on success
 *         -ETIMEDOUT  if `timeout_ms` elapses before all bytes arrive
 *         -EIO        if the CRC does not match
 */
static int wait_for_response(runcam_ctx_t *ctx, uint8_t expected_len, uint32_t timeout_ms) {
    int64_t deadline = k_uptime_get() + timeout_ms;

    ctx->rx_len = 0U;
    memset(ctx->rx_buf, 0, sizeof(ctx->rx_buf));

    while (ctx->rx_len < expected_len) {
        int64_t remaining = deadline - k_uptime_get();

        if (remaining <= 0) {
            LOG_ERR("response timeout (got %u/%u bytes)",
                    ctx->rx_len, expected_len);
            return -ETIMEDOUT;
        }

        /* Wait for the ISR to signal a byte */
        k_sem_take(&ctx->rx_sem, K_MSEC((uint32_t)remaining));
    }

    LOG_HEXDUMP_DBG(ctx->rx_buf, ctx->rx_len, "RX");

    /* CRC check: CRC of all bytes (including CRC byte) should be 0 */
    uint8_t crc = runcam_crc8(ctx->rx_buf, ctx->rx_len);

    if (crc != 0U) {
        LOG_ERR("CRC error: computed 0x%02X over %u bytes", crc, ctx->rx_len);
        return -EIO;
    }

    return 0;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

int runcam_init(runcam_ctx_t *ctx, const struct device *uart_dev)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->uart_dev = uart_dev;
    ctx->recording_status = RUNCAM_STATUS_UNKNOWN;

    k_sem_init(&ctx->rx_sem, 0, RUNCAM_MAX_PACKET_LEN);

    /* Register ISR-driven RX callback */
    int ret = uart_irq_callback_user_data_set(uart_dev, uart_isr_cb, ctx);

    if (ret < 0) {
        LOG_ERR("uart_irq_callback_set failed: %d", ret);
        return ret;
    }

    uart_irq_rx_enable(uart_dev);

    LOG_INF("RunCam driver initialised on %s", uart_dev->name);
    return 0;
}

int runcam_get_device_info(runcam_ctx_t *ctx)
{
    /* Flush any stale bytes */
    k_sem_reset(&ctx->rx_sem);
    ctx->rx_len = 0U;

    /* Send GET_DEVICE_INFO (no action byte) */
    send_packet(ctx, RUNCAM_CMD_GET_DEVICE_INFO, false, 0);

    /*
     * Expected response: 5 bytes
     *   [0] 0xCC
     *   [1] protocol version   (0x01 for v1)
     *   [2] feature bits [7:0]
     *   [3] feature bits [15:8]
     *   [4] CRC
     */
    int ret = wait_for_response(ctx, RUNCAM_RESP_DEVINFO_LEN, RUNCAM_RESPONSE_TIMEOUT_MS);

    if (ret < 0) {
        return ret;
    }

    ctx->info.protocol_version = ctx->rx_buf[1];
    ctx->info.features = (uint16_t)ctx->rx_buf[2]
                       | ((uint16_t)ctx->rx_buf[3] << 8);
    ctx->info.initialized = true;

    LOG_INF("RunCam: proto_ver=0x%02X features=0x%04X",
            ctx->info.protocol_version, ctx->info.features);

    return 0;
}

int runcam_send_control(runcam_ctx_t *ctx, uint8_t action) {
    k_sem_reset(&ctx->rx_sem);
    ctx->rx_len = 0U;

    send_packet(ctx, RUNCAM_CMD_CAMERA_CONTROL, true, action);

    /*
     * The camera sends a 2-byte ACK for CAMERA_CONTROL:
     *   [0] 0xCC
     *   [1] CRC
     *
     * Some older firmware does NOT send an ACK at all.  We attempt to
     * read one but do not treat a timeout as a hard error here.
     */
    int ret = wait_for_response(ctx, RUNCAM_RESP_CTRL_LEN,
                                RUNCAM_RESPONSE_TIMEOUT_MS);

    if (ret == -ETIMEDOUT) {
        LOG_WRN("No ACK for control action 0x%02X (older firmware?)", action);
        return 0;   /* treat as success – command was transmitted */
    }

    return ret;
}

int runcam_start_recording(runcam_ctx_t *ctx) {
    uint8_t action;

    /*
     * Prefer the explicit START command if the camera advertises it.
     * Fall back to a power-button press for cameras that use a toggle.
     */
    if (ctx->info.features & RUNCAM_FEATURE_START_RECORDING) {
        action = RUNCAM_ACTION_START_RECORDING;
    } else {
        LOG_WRN("Camera does not advertise START_RECORDING; "
                "using POWER_BTN toggle");
        action = RUNCAM_ACTION_POWER_BTN;
    }

    int ret = runcam_send_control(ctx, action);

    if (ret == 0) {
        ctx->recording_status = RUNCAM_STATUS_RECORDING;
        LOG_INF("Recording started");
    }

    return ret;
}

int runcam_stop_recording(runcam_ctx_t *ctx) {
    uint8_t action;

    if (ctx->info.features & RUNCAM_FEATURE_STOP_RECORDING) {
        action = RUNCAM_ACTION_STOP_RECORDING;
    } else {
        LOG_WRN("Camera does not advertise STOP_RECORDING; "
                "using POWER_BTN toggle");
        action = RUNCAM_ACTION_POWER_BTN;
    }

    int ret = runcam_send_control(ctx, action);

    if (ret == 0) {
        ctx->recording_status = RUNCAM_STATUS_IDLE;
        LOG_INF("Recording stopped");
    }

    return ret;
}

runcam_recording_status_t runcam_get_recording_status(const runcam_ctx_t *ctx) {
    return ctx->recording_status;
}

const char *runcam_status_str(runcam_recording_status_t status) {
    switch (status) {
    case RUNCAM_STATUS_IDLE:
        return "IDLE";
    case RUNCAM_STATUS_RECORDING:
        return "RECORDING";
    default:
        return "UNKNOWN";
    }
}