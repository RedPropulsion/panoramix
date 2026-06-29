#ifndef OBELIX_STATE_MACHINE_H_
#define OBELIX_STATE_MACHINE_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/smf.h>

#include <mavwrap.h>

#define STATE_MACHINE_STACK_SIZE 10247


#define RX_QUEUE_SIZE 16

K_MSGQ_DEFINE(lora_rx_queue, sizeof(mavlink_message_t), RX_QUEUE_SIZE,
              sizeof(void *));

K_MSGQ_DEFINE(netif_rx_queue, sizeof(mavlink_message_t), RX_QUEUE_SIZE,
              sizeof(void *));


enum obelics_state { STATE_BOOT, STATE_MASTER, STATE_ARMED, STATE_RECEIVER };

enum sm_event {
    SM_EVT_ARM_SENT,       // user sent ARM command
    SM_EVT_ARM_ACK,        // Asterics acknowledged ARM
    SM_EVT_DISARM_SENT,    // user sent DISARM command
    SM_EVT_DISARM_ACK,     // Asterics acknowledged DISARM
    SM_EVT_UNSOLICITED,    // unsolicited telemetry from Asterics
};


struct sfm_state {
  struct smf_ctx ctx;

  enum obelics_state current;
  struct k_thread thread;
  k_thread_stack_t stack[STATE_MACHINE_STACK_SIZE];
  struct file_logger_file *log_file;
  
} __sfm_state;

int  state_machine_init(void);
int  state_machine_start(void);
int  state_machine_post_event(enum sm_event evt, const mavlink_message_t *msg);
int  state_machine_send(const struct device *dev, const mavlink_message_t *msg);  // send gate
enum obelics_state state_machine_get_state(void);

void lora_rx_callback(const struct device *dev,
    const mavlink_message_t *msg, void *user_data);
void netif_rx_callback(const struct device *dev,
    const mavlink_message_t *msg, void *user_data);


enum neopixel_status {
    OK,
    PROCESSING,
    WARN,
    ERR,
    OFF
};

enum neopixel_led {
    COMMS,
    ASTERICS,
    FIX,
    SENSOR_STATUS
};

void led_strip_status(enum neopixel_led, enum neopixel_status);

#endif //OBELIX_STATE_MACHINE_H_