# Sound

Timer-driven buzzer library for Zephyr RTOS. Fire-and-forget: play_sound() returns immediately, timer callback handles sequencing in ISR context.

## Principle

Single k_timer drives note sequencing:

1. Silence buzzer (PWM off)
2. index++ to next note
3. If notes remain: set freq, restart timer for duration
4. Else: call done_cb, reset state

No thread required. Non-blocking API.

## Usage

```c
#include "sound.h"

sound_init(&buzzer_pwm);
play_sound(success_sound, success_sound_len, NULL);
stop_sound();
```

Predefined: success_sound, alert_sound, acknowledge_sound, error_sound