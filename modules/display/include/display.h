#ifndef DISPLAY_H
#define DISPLAY_H

#include <zephyr/kernel.h>
#include <stdint.h>



struct display_data {
    struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    uint8_t satellites;
    uint8_t fix_type;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude_mm;

    

    uint16_t time_size_recv;
    int16_t rssi;
    int8_t snr;

    uint16_t battery_voltage;

    const char *flight_status;
    } gps;    
};

int display_init(void);

int display_string(const char *fmt, ...);

int display_update_row(uint8_t row, const char *fmt, ...);

int display_clear_text(void);

#endif /* DISPLAY_H */