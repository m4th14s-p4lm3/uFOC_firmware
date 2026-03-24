// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_ENCODER_RAW_VALUE 2097151u
#define ENC_MODULO            (MAX_ENCODER_RAW_VALUE + 1u)   // 2097152
#define ENC_HALF_MODULO       (ENC_MODULO / 2u)              // 1048576



typedef struct {
    uint32_t prevous_raw_value;
    uint32_t current_raw_value;

    int64_t  position_ticks;
    int32_t  last_delta;
    uint8_t  valid;

    uint8_t magnetic_pole_pairs;
    uint32_t electrical_offset;
    uint32_t electrical_angle_raw; 
    float electrical_angle;

} encoder_t;

encoder_t init_encoder(uint8_t magnetic_pole_pairs, uint32_t electrical_offset);

void update_encoder(encoder_t* encoder);

double encoder_get_turns(const encoder_t* e);

void update_electrical_offset(encoder_t* encoder, uint32_t new_offset);

uint32_t mt6835_read_raw21(uint8_t *status_out);


#endif
