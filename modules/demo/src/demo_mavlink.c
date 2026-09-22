#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <mavwrap.h>

#include "demo.h"
//#include "obelics.h"

LOG_MODULE_DECLARE(demo, CONFIG_LIB_DEMO_LOG_LEVEL);

#define OBELICS_MAV_SYSTEM_ID     1U
#define OBELICS_MAV_COMPONENT_ID  1U

static void demo_send_command_ack(
    const struct device *dev,
    const mavlink_message_t *request,
    uint16_t command,
    uint8_t result)
{
    mavlink_message_t ack;

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
    case MAV_CMD_OBELICS_LED_OFF:
        demo_neopixel_stop();
        break;

    case MAV_CMD_OBELICS_LED_BOUNCE:
        demo_neopixel_start(DEMO_NEOPIXEL_BOUNCE);
        break;

    case MAV_CMD_OBELICS_LED_SPIN:
        demo_neopixel_start(DEMO_NEOPIXEL_SPIN);
        break;

    case MAV_CMD_OBELICS_LED_BLINK:
        demo_neopixel_start(DEMO_NEOPIXEL_BLINK);
        break;

    case MAV_CMD_OBELICS_SERVO_OFF:
        //demo_servo_stop();
        break;

    case MAV_CMD_OBELICS_SERVO_WIGGLE:
        //demo_servo_start(DEMO_SERVO_WIGGLE);
        break;

    case MAV_CMD_OBELICS_SERVO_SWEEP:
        //demo_servo_start(DEMO_SERVO_SWEEP);
        break;

    case MAV_CMD_OBELICS_SERVO_HELLO:
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
    return mavwrap_start(
        mav_dev,
        demo_mavlink_rx_cb,
        NULL
    );
}