#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include <mavwrap.h>

#include "demo.h"
//#include "obelics.h"

LOG_MODULE_DECLARE(demo, CONFIG_LIB_DEMO_LOG_LEVEL);

#define OBELICS_MAV_SYSTEM_ID     1U
#define OBELICS_MAV_COMPONENT_ID  1U

/* Serialize both packing (MAVLink sequence number) and transport writes. */
K_MUTEX_DEFINE(demo_mavlink_tx_lock);
static const struct device *telemetry_dev;
static uint32_t telemetry_boot_id;

/* COMMAND_LONG/COMMAND_ACK use common wire formats.
 * Keep application command IDs aligned with idefix_mavlink.py. */
enum demo_command_id {
    DEMO_CMD_LED_OFF = 60000,
    DEMO_CMD_LED_BOUNCE = 60001,
    DEMO_CMD_LED_SPIN = 60002,
    DEMO_CMD_LED_BLINK = 60003,
    DEMO_CMD_SERVO_OFF = 60010,
    DEMO_CMD_SERVO_WIGGLE = 60011,
    DEMO_CMD_SERVO_SWEEP = 60012,
    DEMO_CMD_SERVO_HELLO = 60013,
};

static void demo_send_command_ack(
    const struct device *dev,
    const mavlink_message_t *request,
    uint16_t command,
    uint8_t result)
{
    mavlink_message_t ack;

    k_mutex_lock(&demo_mavlink_tx_lock, K_FOREVER);

    mavlink_msg_command_ack_pack(
        OBELICS_MAV_SYSTEM_ID,      /* system_id del mittente: ObeliCS */
        OBELICS_MAV_COMPONENT_ID,   /* component_id del mittente */
        &ack,
        command,                    /* comando a cui stiamo rispondendo */
        result,                     /* MAV_RESULT_ACCEPTED ecc. */
        0U,                         /* progress: non usato */
        0,                          /* result_param2: non usato */
        request->sysid,             /* sistema che ha inviato il comando */
        request->compid             /* componente che ha inviato il comando */
    );

    mavwrap_send_message(dev, &ack);
    k_mutex_unlock(&demo_mavlink_tx_lock);
}

static void demo_mavlink_rx_cb(
    const struct device *dev,
    const mavlink_message_t *msg,
    void *user_data)
{
    ARG_UNUSED(user_data);

    /*
     * Questa callback può ricevere qualsiasi messaggio MAVLink.
     * Per ora gestiamo soltanto COMMAND_LONG.
     */
    if (msg->msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
        return;
    }

    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);

    /*
     * target_system == 0 indica un comando broadcast.
     * In alternativa il comando deve essere indirizzato a ObeliCS.
     */
    if (cmd.target_system != 0U &&
        cmd.target_system != OBELICS_MAV_SYSTEM_ID) {
        return;
    }

    /*
     * Anche target_component == 0 indica broadcast.
     */
    if (cmd.target_component != 0U &&
        cmd.target_component != OBELICS_MAV_COMPONENT_ID) {
        return;
    }

    uint8_t result = MAV_RESULT_ACCEPTED;

    switch (cmd.command) {
    case DEMO_CMD_LED_OFF:
        demo_neopixel_stop();
        break;

    case DEMO_CMD_LED_BOUNCE:
        demo_neopixel_start(DEMO_NEOPIXEL_BOUNCE);
        break;

    case DEMO_CMD_LED_SPIN:
        demo_neopixel_start(DEMO_NEOPIXEL_SPIN);
        break;

    case DEMO_CMD_LED_BLINK:
        demo_neopixel_start(DEMO_NEOPIXEL_BLINK);
        break;

    case DEMO_CMD_SERVO_OFF:
        //demo_servo_stop();
        break;

    case DEMO_CMD_SERVO_WIGGLE:
        //demo_servo_start(DEMO_SERVO_WIGGLE);
        break;

    case DEMO_CMD_SERVO_SWEEP:
        //demo_servo_start(DEMO_SERVO_SWEEP);
        break;

    case DEMO_CMD_SERVO_HELLO:
        //demo_servo_start(DEMO_SERVO_HELLO);
        break;

    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }

    LOG_INF(
        "MAVLink command=%u sender=%u.%u result=%u",
        (unsigned int)cmd.command,
        (unsigned int)msg->sysid,
        (unsigned int)msg->compid,
        (unsigned int)result
    );

    demo_send_command_ack(
        dev,
        msg,
        cmd.command,
        result
    );
}

int demo_mavlink_init(const struct device *mav_dev)
{
    int ret = mavwrap_start(
        mav_dev,
        demo_mavlink_rx_cb,
        NULL
    );
    if (ret == 0) {
        telemetry_boot_id = sys_rand32_get() & INT32_MAX;
        telemetry_dev = mav_dev;
    }
    return ret;
}

void demo_mavlink_publish_lora(const struct demo_lora_stats *stats)
{
    if (!telemetry_dev) {
        return;
    }
    /* One coherent snapshot: identical timestamp for every value. LR_END=1
     * identifies this schema. Receivers publish only COMPLETE snapshots,
     * even if UDP packets arrive out of order. Names fit the 10-byte field. */
    const struct { char name[10]; int32_t value; } values[] = {
        { "LR_BOOT", (int32_t)telemetry_boot_id },
        { "LR_SEQ", (int32_t)stats->sequence },
        { "LR_TX", (int32_t)stats->tx },
        { "LR_RX", (int32_t)stats->rx },
        { "LR_OK", (int32_t)stats->pong },
        { "LR_TO", (int32_t)stats->timeouts },
        { "LR_ERR", (int32_t)stats->errors },
        { "LR_RESULT", stats->result },
        { "LR_RSSI", stats->rssi },
        { "LR_SNR", stats->snr },
        { "LR_RTT", stats->rtt_ms },
        { "LR_AGE", stats->age_ms },
        { "LR_END", 1 },
    };
    uint32_t stamp = k_uptime_get_32();
    for (size_t i = 0; i < ARRAY_SIZE(values); i++) {
        mavlink_message_t msg;
        k_mutex_lock(&demo_mavlink_tx_lock, K_FOREVER);
        mavlink_msg_named_value_int_pack(OBELICS_MAV_SYSTEM_ID,
            OBELICS_MAV_COMPONENT_ID, &msg, stamp,
            values[i].name, values[i].value);
        int ret = mavwrap_send_message(telemetry_dev, &msg);
        k_mutex_unlock(&demo_mavlink_tx_lock);
        if (ret < 0) {
            LOG_WRN("LoRa telemetry interrupted: %d", ret);
            break;
        }
    }
}
