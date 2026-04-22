#ifndef ST3215_H
#define ST3215_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#define ST3215_HEADER         0xFF
#define ST3215_ID_DEFAULT     0x00

// instruction commands
#define ST3215_INST_PING      0x01
#define ST3215_INST_READ      0x02
#define ST3215_INST_WRITE     0x03

// memory registers
#define ST3215_REG_GOAL_POS   0x2A  // address to set angle
#define ST3215_REG_PRESENT_POS 0x38 // address to read angle


// struct to define important parameters of the servo
struct st3215_device {
    const struct device *uart_dev;  // pointer to the UART hardware (PD5, PD6)
    struct gpio_dt_spec enable_pin; // the direction control Pin (PD7)
};

/**
 * @brief Initializes the hardware pins for the servo.
 */
int st3215_init(struct st3215_device *dev);

/**
 * @brief Sends a PING command to verify if the servo is connected.
 * @return 0 if the servo responds correctly, error code otherwise.
 */
int st3215_ping(struct st3215_device *dev);

/**
 * @brief Commands the servo to move to a specific angle.
 * @param angle The target angle (usually 0 to 4095 for these servos).
 */
int st3215_set_angle(struct st3215_device *dev, uint16_t angle);

/**
 * @brief Reads the current physical angle of the servo.
 * @param angle Pointer to store the read angle.
 */
int st3215_get_angle(struct st3215_device *dev, uint16_t *angle);

#endif