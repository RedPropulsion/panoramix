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

extern const Note SOUND_SUCCESS[];
extern const size_t SOUND_SUCCESS_LEN;

extern const Note SOUND_ALERT[];
extern const size_t SOUND_ALERT_LEN;

extern const Note SOUND_ACKNOWLEDGE[];
extern const size_t SOUND_ACKNOWLEDGE_LEN;

extern const Note SOUND_ERROR[];
extern const size_t SOUND_ERROR_LEN;

#endif /* SOUND_H */