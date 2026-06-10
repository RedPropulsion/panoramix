#ifndef ENCODER_INPUT_H
#define ENCODER_INPUT_H

#include <zephyr/kernel.h>

enum encoder_event {
    ENCODER_ROTATE_CW,
    ENCODER_ROTATE_CCW,
    ENCODER_DOUBLE_PRESS,
    ENCODER_PRESS_ROTATE_CW,
    ENCODER_PRESS_ROTATE_CCW,
};

#define ENCODER_MSGQ_SIZE 16

extern struct k_msgq encoder_msgq;

int encoder_input_init(void);

#endif /* ENCODER_INPUT_H */