#include "st3215.h"
#include <zephyr/logging/log.h>

// zepyhr logger for the driver, useful for debugging
LOG_MODULE_REGISTER(st3215_servo, LOG_LEVEL_DBG);

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
    return (uint8_t)(~sum); // NOT bit a bit
}

int st3215_init(struct st3215_device *dev) {
    LOG_INF("Initializing ST3215 Servo Driver...");

    // check if the UART device (PD5/PD6) is ready to be used
    if (!device_is_ready(dev->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV; // Error: No Device||
    }

    // check if the GPIO pin (PD7) is ready
    if (!gpio_is_ready_dt(&dev->enable_pin)) {
        LOG_ERR("Direction control GPIO not ready");
        return -ENODEV;
    }

    // configure the Enable pin as an OUTPUT and set it to LOW initially.
    // LOW means "listen" mode (RX active, TX disabled).
    int ret = gpio_pin_configure_dt(&dev->enable_pin, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure direction control GPIO");
        return ret;
    }

    LOG_INF("ST3215 Servo Driver initialized successfully");
    return 0;
}

/**
 * @brief Sends a PING command to verify if the servo is connected and responsive.
 * @param dev Pointer to the ST3215 device structure.
 * @return 0 if the servo responds correctly, a negative error code otherwise. 
 */
int st3215_ping(struct st3215_device *dev) {
    uint8_t request[6];
    uint8_t response[6];
    
    request[0] = ST3215_HEADER;
    request[1] = ST3215_HEADER;
    request[2] = ST3215_ID_DEFAULT;
    request[3] = 0x02;             // remaining packet  length (2 bytes: ping instruction + checksum)
    request[4] = ST3215_INST_PING; // 0x01
    request[5] = st3215_calc_checksum(request, 6);

    // TX Mode: setting the PD7 enable pin HIGH to send data to the servo
    gpio_pin_set_dt(&dev->enable_pin, 1);

    // send the ping packet byte by byte through UART
    for (int i = 0; i < 6; i++) {
        uart_poll_out(dev->uart_dev, request[i]);
    }

    k_usleep(50); // wait to ensure the packet is sent before switching to receive mode

    // RX Mode: setting the PD7 pin back to 0 to listen for the servo's response
    gpio_pin_set_dt(&dev->enable_pin, 0);

    // response handling with timeout
    int bytes_received = 0;
    int attempts = 0;
    while (bytes_received < 6 && attempts < 1000) {
        unsigned char c;
        if (uart_poll_in(dev->uart_dev, &c) == 0) {
            response[bytes_received++] = c;
        } else {
            attempts++;
            k_usleep(10);
        }
    }
    // response structure: [0xFF, 0xFF, ID, Len, Status, Checksum]

    // response checks
    if (bytes_received < 6) {
        LOG_ERR("PING failed: timeout receiving data (only %d bytes received)", bytes_received);
        return -ETIMEDOUT;
    }
    if (response[0] != 0xFF || response[1] != 0xFF) {
        LOG_ERR("PING failed: invalid headers!");
        return -EIO;
    }
    if (response[2] != ST3215_ID_DEFAULT) {
        LOG_ERR("PING failed: incorrect ID!");
        return -EIO;
    }
    if (response[4] != 0) { // see doc page 3 for the meaning of the status byte
        LOG_WRN("PING response error! Status Byte: 0x%02X", response[4]);
    }
    uint8_t rx_checksum = st3215_calc_checksum(response, 6);
    if (rx_checksum != response[5]) {
        LOG_ERR("PING failed: checksum response not valid! (calculated: 0x%02X, Received: 0x%02X)", rx_checksum, response[5]);
        return -EBADMSG;
    }

    return 0;
}

/**
 * @brief Sets the angle of the ST3215 servo.
 * @param dev Pointer to the ST3215 device.
 * @param angle Desired angle (0-4095).
 * @return 0 on success, a negative value on error.
 */
int st3215_set_angle(struct st3215_device *dev, uint16_t angle) {
    uint8_t packet[9];
    // since the angle input parameter is 16-bit, we need to split it into two bytes for the packet
    uint8_t angle_low = angle & 0xFF;
    uint8_t angle_high = (angle >> 8) & 0xFF;

    packet[0] = ST3215_HEADER;
    packet[1] = ST3215_HEADER;
    packet[2] = ST3215_ID_DEFAULT;   
    packet[3] = 0x05;   // remaining packet length (5 bytes: Instruction + Address + Data Length + 2 bytes of angle + Checksum)
    packet[5] = ST3215_REG_GOAL_POS;
    packet[6] = angle_low;
    packet[7] = angle_high;
    packet[8] = st3215_calc_checksum(packet, 9);

    LOG_INF("Sending angle command: %d (Checksum: 0x%02X)", angle, packet[8]);

    gpio_pin_set_dt(&dev->enable_pin, 1); // TX Mode to send data

    for (int i = 0; i < 9; i++) {
        uart_poll_out(dev->uart_dev, packet[i]); // writes byte by byte to the UART
    }

    k_usleep(100);
    gpio_pin_set_dt(&dev->enable_pin, 0); // back to RX mode after sending

    return 0;
}

/**
    * @brief Reads the current angle of the ST3215 servo.
    * @param dev Pointer to the ST3215 device.
    * @param angle Pointer to store the read angle.
    * @return 0 on success, a negative value on error.
*/
int st3215_get_angle(struct st3215_device *dev, uint16_t *angle) {
    uint8_t request[8];
    uint8_t response[8];
    
    // preparing the read packet to request the current angle from the servo
    request[0] = ST3215_HEADER;
    request[1] = ST3215_HEADER;
    request[2] = ST3215_ID_DEFAULT;
    request[3] = 0x04; // remaining packet length (4 bytes: Instruction + Address + Data Length + Checksum)
    request[4] = ST3215_INST_READ;
    request[5] = ST3215_REG_PRESENT_POS;
    request[6] = 0x02; // we want to read 2 bytes (the angle is 16-bit) starting from the position specified in request[5]
    request[7] = st3215_calc_checksum(request, 8);

    gpio_pin_set_dt(&dev->enable_pin, 1); // TX Mode to send data
    for (int i = 0; i < 8; i++) {
        uart_poll_out(dev->uart_dev, request[i]);
    }
    
    // sleep to make sure the data is sent before switching to receive mode
    k_usleep(50); 

    gpio_pin_set_dt(&dev->enable_pin, 0); // RX Mode to receive data

    // handle servo's answer and eventual timeout
    int bytes_received = 0;
    int attempts = 0;
    const int MAX_ATTEMPTS = 1000; // timeout after 1000 attempts

    while (bytes_received < 8 && attempts < MAX_ATTEMPTS) {
        unsigned char c;
        if (uart_poll_in(dev->uart_dev, &c) == 0) {
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