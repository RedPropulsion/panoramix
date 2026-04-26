#include "sound.h"
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_DECLARE(sound);

#define NOTE_REST SOUND_REST

#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_AS4 466
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988

static const struct pwm_dt_spec *sound_pwm;

static struct {
    const Note *notes;
    size_t count;
    size_t index;
    sound_done_cb done_cb;
} sound_state;

static struct k_timer sound_timer;

static void sound_note_set(uint32_t freq_hz, uint32_t dur_ms)
{
    if (freq_hz == NOTE_REST || freq_hz == 0) {
        pwm_set_dt(sound_pwm, sound_pwm->period, 0);
    } else {
        uint32_t period_ns = NSEC_PER_SEC / freq_hz;
        pwm_set_dt(sound_pwm, period_ns, period_ns / 2U);
    }
    k_timer_start(&sound_timer, K_MSEC(dur_ms), K_NO_WAIT);
}

static void sound_timer_handler(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    pwm_set_dt(sound_pwm, sound_pwm->period, 0);

    if (sound_state.index >= sound_state.count) {
        sound_done_cb cb = sound_state.done_cb;
        memset(&sound_state, 0, sizeof(sound_state));
        if (cb) {
            cb();
        }
        return;
    }

    const Note *n = &sound_state.notes[sound_state.index++];
    sound_note_set(n->freq_hz, n->dur_ms);
}

void sound_init(const struct pwm_dt_spec *pwm)
{
    sound_pwm = pwm;
    k_timer_init(&sound_timer, sound_timer_handler, NULL);
}

void stop_sound(void)
{
    k_timer_stop(&sound_timer);
    if (sound_pwm) {
        pwm_set_dt(sound_pwm, sound_pwm->period, 0);
    }
    memset(&sound_state, 0, sizeof(sound_state));
}

void play_sound(const Note *notes, size_t count, sound_done_cb done_cb)
{
    // k_timer_stop(&sound_timer);
    stop_sound();
    memset(&sound_state, 0, sizeof(sound_state));
    sound_state.notes = notes;
    sound_state.count = count;
    sound_state.done_cb = done_cb;
    sound_timer_handler(NULL);
}


const Note success_sound[] = {
    {NOTE_G5, SOUND_EIGHTH}, {NOTE_REST, SOUND_EIGHTH}, {NOTE_A5, SOUND_QUARTER},
};
const size_t success_sound_len = ARRAY_SIZE(success_sound);

const Note alert_sound[] = {
    {NOTE_E5, 60}, {NOTE_REST, 60}, {NOTE_E5, 60}, {NOTE_REST, 60},
    {NOTE_E5, 60}, {NOTE_REST, 60},
};
const size_t alert_sound_len = ARRAY_SIZE(alert_sound);

const Note acknowledge_sound[] = {
    {NOTE_A5, SOUND_QUARTER},
};
const size_t acknowledge_sound_len = ARRAY_SIZE(acknowledge_sound);

const Note error_sound[] = {
    {NOTE_E4, SOUND_HALF}, {NOTE_REST, SOUND_EIGHTH}, {NOTE_D4, SOUND_HALF},
};
const size_t error_sound_len = ARRAY_SIZE(error_sound);