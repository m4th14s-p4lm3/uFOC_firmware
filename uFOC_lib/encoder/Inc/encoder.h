// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_ENCODER_RAW_VALUE   2097151u
#define ENC_MODULO              (MAX_ENCODER_RAW_VALUE + 1u)   // 2097152
#define ENC_HALF_MODULO         (ENC_MODULO / 2u)              // 1048576

/* Maximum capacity of the velocity circular buffer.
 * num_samples must be <= this value. */
#define ENCODER_MAX_VEL_SAMPLES 64u


typedef struct {
    bool invert_dir;

    uint32_t prevous_raw_value;
    uint32_t current_raw_value;


    float angular_velocity;
    float angular_velocity_ewma;
    float angular_velocity_ewma_alpha;

    float ewma_value;
    float ewma_previous_value;
    float ewma_delta;
    float ewma_alpha;

    int64_t  position_ticks;
    int32_t  last_delta;
    uint8_t  valid;

    uint8_t magnetic_pole_pairs;
    uint32_t electrical_offset;
    uint32_t electrical_angle_raw;
    float electrical_angle;

    /* CRC diagnostika */
    uint32_t last_good_raw;
    uint32_t crc_error_count;

} encoder_t;

encoder_t init_encoder(uint8_t magnetic_pole_pairs, uint32_t electrical_offset, bool invert_dir);

void update_encoder(encoder_t* encoder);

float encoder_get_turns(const encoder_t* e);

void update_electrical_offset(encoder_t* encoder, uint32_t new_offset);

uint32_t mt6835_read_raw21(uint8_t *status_out, bool *crc_ok);

float get_angular_velocity_raw(const encoder_t* encoder, float dt);

float get_electrical_angle(encoder_t* encoder);

float get_angular_velocity(encoder_t* encoder);


float get_velocity_moving_average(const encoder_t* encoder);

void mt6835_init();


void calibrate_electrical_offset(encoder_t* encoder);


#endif
