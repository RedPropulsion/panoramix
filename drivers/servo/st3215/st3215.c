#define DT_DRV_COMPAT waveshare_st3215
#define ST3215_HEADER          0xFF
#define ST3215_ID_DEFAULT      0x01
#define ST3215_REG_GOAL_POS    0x2A
#define ST3215_REG_PRESENT_POS 0x38
#define ST3215_INST_WRITE      0x03
#define ST3215_INST_READ       0x02
#define ST3215_INST_PING       0x01

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/servo.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(st3215_servo, LOG_LEVEL_DBG);

struct st3215_config {
    const struct device *uart_dev;
};

struct st3215_data {
    // what kind of (runtime) data do we store here?
};

/** @brief Calculates the checksum of the ST3215 packet. The checksum is the bitwise NOT of the sum of the bytes starting from the ID.
 * @param packet The packet for which to calculate the checksum.
 * @param total_length The total length of the packet (including header and checksum).
 * @return The calculated checksum byte.
 */
static uint8_t st3215_calc_checksum(uint8_t *packet, uint8_t total_length) {
    uint32_t sum = 0;
    // sums all the bytes starting from the ID (index 2) up to the last byte before the checksum
    for (int i = 2; i < total_length - 1; i++) {
        sum += packet[i];
    }
    return (uint8_t)(~sum); // NOT bit by bit of the sum
}

/**
 * @brief Sends the raw angle command to the ST3215 servo. This function constructs the command packet according to the servo's protocol, sends it over UART, and handles the necessary GPIO pin for switching between TX and RX modes.
 * @param dev Pointer to the ST3215 device.
 * @param angle Desired angle in the servo's 12-bit scale (0-4095)
 */
int st3215_send_angle_raw(const struct device *dev, uint16_t angle) {
    const struct st3215_config *config = dev->config;

    uint8_t packet[9];
    // since the angle input parameter is 16-bit, we need to split it into two bytes for the packet
    uint8_t angle_low = angle & 0xFF;
    uint8_t angle_high = (angle >> 8) & 0xFF;

    packet[0] = ST3215_HEADER;
    packet[1] = ST3215_HEADER;
    packet[2] = ST3215_ID_DEFAULT;
    packet[3] = 0x05;   // remaining packet length (5 bytes: Instruction + Address + Data Length + 2 bytes of angle + Checksum)
    packet[4] = ST3215_INST_WRITE;
    packet[5] = ST3215_REG_GOAL_POS;
    packet[6] = angle_low;
    packet[7] = angle_high;
    packet[8] = st3215_calc_checksum(packet, 9);

    LOG_INF("Sending angle command: %d (Checksum: 0x%02X)", angle, packet[8]);
   
    for (int i = 0; i < 9; i++) {
        uart_poll_out(config->uart_dev, packet[i]); // writes byte by byte to the UART
    }

    k_usleep(100);
    return 0;
}

static int st3215_set_position(const struct device *dev, int32_t angle_mdeg) {
    // convert the angle from millidegrees to the servo's 12-bit scale (0-4095)
    uint16_t raw_angle = (uint16_t)((angle_mdeg * 4095) / 360000);
    return st3215_send_angle_raw(dev, raw_angle);
}

/**
    * @brief Reads the current angle of the ST3215 servo.
    * @param dev Pointer to the ST3215 device.
    * @param angle Pointer to store the read angle.
    * @return 0 on success, a negative value on error.
*/
int st3215_get_angle_raw(const struct device *dev, uint16_t *angle) {
    const struct st3215_config *config = dev->config;
    uint8_t request[8];
    uint8_t response[8];
    unsigned char temp;

    // a "flush" of the UART receive buffer to eventually discard old data
    while (uart_poll_in(config->uart_dev, &temp) == 0) {
        // just receives the byte and does nothing
    }
    
    // preparing the read packet to request the current angle from the servo
    request[0] = ST3215_HEADER;
    request[1] = ST3215_HEADER;
    request[2] = ST3215_ID_DEFAULT;
    request[3] = 0x04; // remaining packet length (4 bytes: Instruction + Address + Data Length + Checksum)
    request[4] = ST3215_INST_READ;
    request[5] = ST3215_REG_PRESENT_POS;
    request[6] = 0x02; // we want to read 2 bytes (the angle is 16-bit) starting from the position specified in request[5]
    request[7] = st3215_calc_checksum(request, 8);

    for (int i = 0; i < 8; i++) {
        uart_poll_out(config->uart_dev, request[i]);
    }
    
    // sleep to make sure the data is sent before switching to receive mode
    k_usleep(50); 

    // handle servo's answer and eventual timeout
    int bytes_received = 0;
    int attempts = 0;
    const int MAX_ATTEMPTS = 1000; // timeout after 1000 attempts

    while (bytes_received < 8 && attempts < MAX_ATTEMPTS) {
        unsigned char c;
        if (uart_poll_in(config->uart_dev, &c) == 0) {
            response[bytes_received++] = c;
        } else {
            attempts++;
            k_usleep(10); // waits 10 microseconds before trying again to receive data
        }
    }

    if (bytes_received < 8) {
        LOG_ERR("Error: timeout receiving data (only %d bytes received)", bytes_received);
        return -ETIMEDOUT;
    }
    // response structure: [0xFF, 0xFF, ID, Len, Status, Val_LOW, Val_HIGH, Checksum]
    
    // response checks    
    if (response[0] != 0xFF || response[1] != 0xFF) {
        LOG_ERR("Errore: invalid headers!");
        return -EIO;
    }
    if (response[2] != ST3215_ID_DEFAULT) {
    LOG_ERR("Errore: Risposta da ID inatteso (Ricevuto: %d)", response[2]);
    return -EIO;
    }
    if (response[4] != 0) { // see doc page 3 for the meaning of the status byte
        LOG_WRN("Response Error! Status Byte: 0x%02X", response[4]);
    }

    // checks the checksum of the received packet to verify data integrity
    uint8_t rx_checksum = st3215_calc_checksum(response, 8);
    if (rx_checksum != response[7]) {
        LOG_ERR("Error: checksum response not valid! (calculated: 0x%02X, Received: 0x%02X)", 
                rx_checksum, response[7]);
        return -EBADMSG;
    }

    // reconstruction of the angle (combining the two bytes)
    *angle = response[5] | (response[6] << 8);

    return 0;
}

static int st3215_get_position(const struct device *dev, int32_t *angle_mdeg) {
    uint16_t angle;
    int ret = st3215_get_angle_raw(dev, &angle);
    if (ret < 0) {
        return ret;
    }
    // convert the angle from the servo's 12-bit scale back to millidegrees
    *angle_mdeg = (int32_t)((angle * 360000) / 4095);
    return 0;
}

static int st3215_init(const struct device *dev) {
    const struct st3215_config *config = dev->config;

    if (!device_is_ready(config->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }

    LOG_INF("ST3215 Driver initialized on %s", config->uart_dev->name);
    return 0;
}

static DEVICE_API(servo, st3215_api) = {
    .set_position = st3215_set_position,
    .get_position = st3215_get_position,
};

#define ST3215_DEFINE(inst)                                        \
    static struct st3215_data st3215_data_##inst;                  \
                                                                   \
    static const struct st3215_config st3215_config_##inst = {     \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),              \
    };                                                             \
                                                                   \
    DEVICE_DT_INST_DEFINE(inst,                                    \
                          st3215_init,                             \
                          NULL,                                    \
                          &st3215_data_##inst,                     \
                          &st3215_config_##inst,                   \
                          POST_KERNEL,                             \
                          CONFIG_SERVO_INIT_PRIORITY,              \
                          &st3215_api);

DT_INST_FOREACH_STATUS_OKAY(ST3215_DEFINE)