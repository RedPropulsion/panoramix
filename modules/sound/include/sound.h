#ifndef SOUND_H
#define SOUND_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#define SOUND_REST 0

#define SOUND_SIXTEENTH  75
#define SOUND_EIGHTH     150
#define SOUND_QUARTER    300
#define SOUND_HALF       600
#define SOUND_WHOLE      1200

typedef struct { uint32_t freq_hz; uint32_t dur_ms; } Note;

typedef void (*sound_done_cb)(void);

void sound_init(const struct pwm_dt_spec *pwm);

void play_sound(const Note *notes, size_t count, sound_done_cb done_cb);

void stop_sound(void);

extern const Note success_sound[];
extern const size_t success_sound_len;

extern const Note alert_sound[];
extern const size_t alert_sound_len;

extern const Note acknowledge_sound[];
extern const size_t acknowledge_sound_len;

extern const Note error_sound[];
extern const size_t error_sound_len;

#endif /* SOUND_H */