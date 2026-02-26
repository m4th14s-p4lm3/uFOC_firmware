// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t prevous_raw_value;
    uint32_t current_raw_value;

    int64_t  position_ticks;
    int32_t  last_delta;
    uint8_t  valid;
} encoder_t;

uint32_t mt6835_read_raw21(uint8_t *status_out);

void measure_encoder(encoder_t* encoder);

double encoder_get_turns(const encoder_t* e);

#endif
