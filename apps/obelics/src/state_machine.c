#include <zephyr/kernel.h>
#include <zephyr/arch/arch_interface.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>

#include <file_logger.h>
#include <menu.h>

#include "state_machine.h"


LOG_MODULE_REGISTER(StateMachine);

#define STATE_MACHINE_PRIORITY 5

static const struct device *mavlink_lora =
    DEVICE_DT_GET(DT_NODELABEL(mavlink_lora));

static const struct device *mavlink_udp =
    DEVICE_DT_GET(DT_NODELABEL(mavlink_netif));


#define STRIP_NODE  DT_NODELABEL(led_strip)
#define NUM_LEDS    DT_PROP(STRIP_NODE, chain_length)

static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static const struct gpio_dt_spec neopixel_en =
    GPIO_DT_SPEC_GET(DT_NODELABEL(neopixel_en), gpios);

struct led_rgb pixels[NUM_LEDS] = {0};


void boot_entry();
enum smf_state_result boot_run();
void boot_exit();

void master_entry();
enum smf_state_result master_run();
void master_exit();

void armed_entry();
enum smf_state_result armed_run();
void armed_exit();

void receiver_entry();
enum smf_state_result receiver_run();
void receiver_exit();

const struct smf_state obelics_states[] = {
  [STATE_BOOT] =
      SMF_CREATE_STATE(boot_entry, boot_run, boot_exit, NULL, NULL),
  [STATE_MASTER] =
      SMF_CREATE_STATE(master_entry, master_run, master_exit, NULL, NULL),
  [STATE_ARMED] =
      SMF_CREATE_STATE(armed_entry, armed_run, armed_exit, NULL, NULL),
  [STATE_RECEIVER] = SMF_CREATE_STATE(receiver_entry, receiver_run,
                                        receiver_exit, NULL, NULL)
};

int state_machine_init() {
     __sfm_state.current = STATE_BOOT; 
     return 0;
};

void lora_rx_callback(const struct device *dev,
                             const mavlink_message_t *msg, void *user_data) {
  int ret = k_msgq_put(&lora_rx_queue, msg, K_NO_WAIT);
  if (ret < 0) {
    LOG_ERR("Queue overflow, clearing...");
    k_msgq_purge(&lora_rx_queue);
    k_msgq_put(&lora_rx_queue, msg, K_NO_WAIT);
  }

  struct mavwrap_stats stats;
  ret = mavwrap_get_stats(dev, &stats);
  if (ret < 0) {
    LOG_ERR("\tCouldn't get LORA stats: %s", strerror(-ret));
  } else {
    LOG_DBG("\tRSSI: %d\tSNR: %d", stats.rx_rssi, stats.rx_snr);
    if (stats.tx_errors) {
      LOG_WRN("\ttx errors: %d", stats.tx_errors);
    }
  }

  menu_update_lora_stats(stats.rx_packets, stats.tx_packets, stats.rx_rssi,
                         stats.rx_snr);

  mavwrap_send_message(mavlink_udp, msg);
};

void netif_rx_callback(const struct device *dev,
                              const mavlink_message_t *msg, void *user_data) {
    //TODO: add packet dropping pabes on state
    int ret = k_msgq_put(&netif_rx_queue, msg, K_NO_WAIT);
    if (ret < 0) {
      LOG_ERR("Queue overflow, clearing...");
      k_msgq_purge(&lora_rx_queue);
      k_msgq_put(&lora_rx_queue, msg, K_NO_WAIT);
    }

    state_machine_send(mavlink_lora, msg);
};


void led_strip_status(enum neopixel_led led, enum neopixel_status status){
  switch (status) {
    case OK:
      break;
    case PROCESSING:
      break;
    case WARN:
      break;
    case ERR:
      break;
    case OFF:
     pixels[led].r = 0;
     pixels[led].g = 0;
     pixels[led].b = 0;
      break;
  }   
};