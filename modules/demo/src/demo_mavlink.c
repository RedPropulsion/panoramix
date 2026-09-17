#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <mavwrap.h>
#include "demo.h"
LOG_MODULE_DECLARE(demo, CONFIG_LIB_DEMO_LOG_LEVEL);
static void demo_send_ack(const struct device *dev, uint8_t action, uint8_t result)
{
    mavlink_message_t msg;
    mavlink_msg_obelics_demo_ack_pack(1, 1, &msg, action, result,
                                      demo_active() ? 1u : 0u);
    mavwrap_send_message(dev, &msg);
}
static void demo_mavlink_rx_cb(const struct device *dev,
                               const mavlink_message_t *msg,
                               void *user_data)
{
    ARG_UNUSED(user_data);
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_OBELICS_DEMO_COMMAND: {
        mavlink_obelics_demo_command_t cmd;
        mavlink_msg_obelics_demo_command_decode(msg, &cmd);
        switch (cmd.action) {
        case DEMO_ACTION_OFF:    demo_off();                            break;
        case DEMO_ACTION_BOUNCE: demo_neopixel_start(DEMO_NEOPIXEL_BOUNCE); break;
        case DEMO_ACTION_SPIN:   demo_neopixel_start(DEMO_NEOPIXEL_SPIN);   break;
        case DEMO_ACTION_BLINK:  demo_neopixel_start(DEMO_NEOPIXEL_BLINK);  break;
        case DEMO_ACTION_WIGGLE: demo_servo_start(DEMO_SERVO_WIGGLE);   break;
        case DEMO_ACTION_SWEEP:  demo_servo_start(DEMO_SERVO_SWEEP);    break;
        case DEMO_ACTION_HELLO:  demo_servo_start(DEMO_SERVO_HELLO);    break;
        default:
            demo_send_ack(dev, cmd.action, 1);
            return;
        }
        demo_send_ack(dev, cmd.action, 0);
        break;
    }
    default:
        break;
    }
}

int demo_mavlink_init(const struct device *mav_dev)
{
    return mavwrap_start(mav_dev, demo_mavlink_rx_cb, NULL);
}