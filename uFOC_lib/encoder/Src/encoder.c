// encoder.c
#include "encoder.h"
#include "stm32f3xx_hal.h"
#include <math.h>

#define ENC_CS_GPIO_Port GPIOA
#define ENC_CS_Pin       GPIO_PIN_5

// #define MAX_ENCODER_RAW_VALUE 2097151u
// #define ENC_MODULO            (MAX_ENCODER_RAW_VALUE + 1u)   // 2097152
// #define ENC_HALF_MODULO       (ENC_MODULO / 2u)              // 1048576


extern SPI_HandleTypeDef hspi3;

/* -------------------------------------------------------------------------- */
/* CRC-8 pro MT6835                                                           */
/* -------------------------------------------------------------------------- */

/* Nastav na 0 pro dočasné vypnutí CRC (debug). */
#define MT6835_CRC_ENABLED  1

/* MT6835 CRC-8, polynomial 0x07, init 0x00.
 * Pokryté bajty: rx[2], rx[3], rx[4] (reg 0x003–0x005).
 * Pokud crc_error_count stále roste, zkus změnit CRC_INIT nebo rozsah
 * bajtů — ověř v datasheetu MT6835 sekci "CRC Check". */
#define MT6835_CRC_INIT     0x00u
#define MT6835_CRC_POLY     0x07u

static uint8_t crc8_mt6835(const uint8_t *data, uint8_t len)
{
    uint8_t crc = MT6835_CRC_INIT;
    for (uint8_t i = 0u; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1u) ^ MT6835_CRC_POLY);
            } else {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }
    return crc;
}

static HAL_StatusTypeDef spi3_configure_for_encoder(void)
{
    if (hspi3.Init.CLKPolarity == SPI_POLARITY_HIGH &&
        hspi3.Init.CLKPhase == SPI_PHASE_2EDGE) {
        return HAL_OK;
    }

    HAL_SPI_DeInit(&hspi3);

    hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;

    return HAL_SPI_Init(&hspi3);
}


encoder_t init_encoder(uint8_t magnetic_pole_pairs, uint32_t electrical_offset, bool invert_dir){
    encoder_t encoder;
    uint8_t status_out;
    bool crc_ok = false;
    uint32_t new_raw_value = mt6835_read_raw21(&status_out, &crc_ok);
    
    encoder.invert_dir = invert_dir;

    encoder.current_raw_value = new_raw_value;
    encoder.prevous_raw_value = new_raw_value;
    

    encoder.angular_velocity = 0.0f;
    encoder.angular_velocity_ewma = 0.0f;
    encoder.angular_velocity_ewma_alpha = 0.5f;



    encoder.ewma_previous_value = (float)new_raw_value;
    encoder.ewma_value = (float)new_raw_value;
    encoder.ewma_alpha = 0.8f;
    // encoder.ewma_alpha = 0.05f;
    encoder.ewma_delta = 0.0f;
    
    encoder.position_ticks = new_raw_value;
    encoder.last_delta = 0;
    encoder.valid = 1;
    
    encoder.magnetic_pole_pairs = magnetic_pole_pairs;
    encoder.electrical_offset = electrical_offset;
    encoder.electrical_angle_raw = 0;
    encoder.electrical_angle = 0.0f;

    encoder.last_good_raw = new_raw_value;
    encoder.crc_error_count = 0u;

    /* Velocity moving-average buffer — default window = 8 samples */
    encoder.num_samples   = 8u;
    encoder.vel_buf_head  = 0u;
    encoder.vel_buf_count = 0u;
    for (uint8_t i = 0u; i < ENCODER_MAX_VEL_SAMPLES; i++) {
        encoder.vel_buf[i] = 0.0f;
    }
    // spi3_configure_for_encoder();

    return encoder;
}


void update_encoder(encoder_t* encoder){
    if (!encoder) return;

    uint8_t status_out;
    bool crc_ok = false;
    uint32_t new_raw_value = mt6835_read_raw21(&status_out, &crc_ok);

    if (!crc_ok) {
        encoder->crc_error_count++;
        /* Použij poslední dobrou hodnotu — motor pokračuje bez záškubu.
         * delta = 0, takže velocity estimace "zamrzne" na jednu periodu. */
        new_raw_value = encoder->last_good_raw;
    } else {
        encoder->last_good_raw = new_raw_value;
    }
        
    encoder->prevous_raw_value = encoder->current_raw_value;
    encoder->current_raw_value = new_raw_value;
    
    int32_t delta = (int32_t)encoder->current_raw_value - (int32_t)encoder->prevous_raw_value;
    
    if (delta > (int32_t)ENC_HALF_MODULO) {
        delta -= (int32_t)ENC_MODULO;
    } else if (delta < -(int32_t)ENC_HALF_MODULO) {
        delta += (int32_t)ENC_MODULO;
    }
    
    encoder->last_delta = delta;
    encoder->position_ticks += (int64_t)delta;
    


    
    
    
    encoder->ewma_value = encoder->ewma_alpha * encoder->position_ticks + (1.0f - encoder->ewma_alpha) * encoder->ewma_value;
    encoder->ewma_delta = encoder->ewma_value - encoder->ewma_previous_value;
    encoder->ewma_previous_value = encoder->ewma_value;
    
    encoder->angular_velocity = get_angular_velocity(encoder, 0.0002f); // unit is RADIANS - NOTE: Add "dt" property to init!
    encoder->angular_velocity_ewma = encoder->angular_velocity_ewma_alpha * encoder->angular_velocity + 
                                        (1 - encoder->angular_velocity_ewma_alpha) * encoder->angular_velocity_ewma;
    


    int64_t mech_raw_i = encoder->position_ticks % (int64_t)ENC_MODULO;
    if (mech_raw_i < 0) {
        mech_raw_i += (int64_t)ENC_MODULO;
    }

    /* mech_raw_i ∈ [0, 2^21-1], × pole_pairs (11) max = ~23M < INT32_MAX,
     * ale int64 pro jistotu. */
    int64_t mech_elec_raw_i = (mech_raw_i * (int64_t)encoder->magnetic_pole_pairs)
                              % (int64_t)ENC_MODULO;

    int64_t elec_raw_i;
    if (mech_elec_raw_i >= (int64_t)encoder->electrical_offset) {
        elec_raw_i = mech_elec_raw_i - (int64_t)encoder->electrical_offset;
    } else {
        elec_raw_i = mech_elec_raw_i + (int64_t)ENC_MODULO - (int64_t)encoder->electrical_offset;
    }

    encoder->electrical_angle_raw = (uint32_t)elec_raw_i;
    encoder->electrical_angle =
        (2.0f * 3.14159265359f * (float)elec_raw_i) / (float)ENC_MODULO;

    if (encoder->invert_dir){
        encoder->electrical_angle = -(encoder->electrical_angle - 2 * M_PI);
    }

    /* Push the latest angular_velocity into the circular buffer.
     * Clamp num_samples to a valid range to guard against misconfiguration. */
    uint8_t n = encoder->num_samples;
    if (n == 0u) n = 1u;
    if (n > ENCODER_MAX_VEL_SAMPLES) n = ENCODER_MAX_VEL_SAMPLES;

    encoder->vel_buf[encoder->vel_buf_head] = encoder->angular_velocity;
    encoder->vel_buf_head = (uint8_t)((encoder->vel_buf_head + 1u) % n);
    if (encoder->vel_buf_count < n) {
        encoder->vel_buf_count++;
    }
}

void update_electrical_offset(encoder_t* encoder, uint32_t new_offset){
    if (!encoder) return;
    encoder->electrical_offset = new_offset;
}

double encoder_get_turns(const encoder_t* e) {
    return (double)e->position_ticks / (double)ENC_MODULO;
}



void mt6835_init(){
    spi3_configure_for_encoder();
}
uint32_t mt6835_read_raw21(uint8_t *status_out, bool *crc_ok)
{
    // Burst command (0xA) + start address 0x003
    // Frame: [cmd|addr][dummy x4] → rx: [don't care x2][ANGLE[20:13]][ANGLE[12:5]][ANGLE[4:0]+STATUS][CRC]
    uint8_t tx[6] = { (uint8_t)(0xA << 4), 0x03, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rx[6] = {0};

    if (crc_ok) {
        *crc_ok = false;
    }

    // if (spi3_configure_for_encoder() != HAL_OK) {
    //     if (status_out) {
    //         *status_out = 0xFF;
    //     }
    //     return 0u;
    // }

    ENC_CS_GPIO_Port->BSRR = (uint32_t)ENC_CS_Pin << 16; // CS low
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, sizeof(tx), 100);
    ENC_CS_GPIO_Port->BSRR = ENC_CS_Pin;                  // CS high

    // Datasheet: 0x003 = ANGLE[20:13], 0x004 = ANGLE[12:5],
    //            0x005 = ANGLE[4:0] + STATUS[2:0], 0x006 = CRC
    uint32_t raw =
        ((uint32_t)rx[2] << 13) |
        ((uint32_t)rx[3] << 5)  |
        ((uint32_t)rx[4] >> 3);

    raw &= 0x1FFFFFu; // 21 bit

    if (status_out) {
        *status_out = (uint8_t)(rx[4] & 0x07u); // STATUS[2:0]
    }

    if (crc_ok) {
        uint8_t computed = crc8_mt6835(&rx[2], 3u);
        *crc_ok = (computed == rx[5]);
    }

    return raw;
}

/* Returns the moving average of the last num_samples angular-velocity readings.
 * The sign is flipped when invert_dir is set, consistent with the logical
 * motor direction convention used for electrical_angle. */
float get_velocity_moving_average(const encoder_t* encoder)
{
    if (!encoder) return 0.0f;

    uint8_t count = encoder->vel_buf_count;
    if (count == 0u) return 0.0f;

    float sum = 0.0f;
    for (uint8_t i = 0u; i < count; i++) {
        sum += encoder->vel_buf[i];
    }

    float avg = sum / (float)count;

    if (encoder->invert_dir) {
        avg = -avg;
    }

    return avg;
}

// angular velocity in radians per second
float get_angular_velocity(const encoder_t* encoder, float dt){
    if (!encoder || dt <= 0.0f) return 0.0f;

    float delta_ticks = encoder->ewma_delta;
    float velocity = (delta_ticks / dt) * ((2.0f * 3.14159265359f) / (float)ENC_MODULO);
    return velocity;
}
