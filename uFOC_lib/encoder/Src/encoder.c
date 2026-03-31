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


encoder_t init_encoder(uint8_t magnetic_pole_pairs, uint32_t electrical_offset){
    encoder_t encoder;
    uint8_t status_out;
    uint32_t new_raw_value = mt6835_read_raw21(&status_out);
    
    encoder.current_raw_value = new_raw_value;
    encoder.prevous_raw_value = new_raw_value;
    
    encoder.ewma_previous_value = (float)new_raw_value;
    encoder.ewma_value = (float)new_raw_value;
    encoder.ewma_alpha = 0.9f;
    encoder.ewma_delta = 0.0f;
    
    encoder.position_ticks = new_raw_value;
    encoder.last_delta = 0;
    encoder.valid = 1;
    
    encoder.magnetic_pole_pairs = magnetic_pole_pairs;
    encoder.electrical_offset = electrical_offset;
    encoder.electrical_angle_raw = 0;
    encoder.electrical_angle = 0.0f;
    
    return encoder;
}


void update_encoder(encoder_t* encoder){
    if (!encoder) return;
    
    uint8_t status_out;
    uint32_t new_raw_value = mt6835_read_raw21(&status_out);
        
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
    
    encoder->ewma_value = encoder->ewma_alpha * (float)encoder->position_ticks + (1.0f - encoder->ewma_alpha) * encoder->ewma_value;
    encoder->ewma_delta = encoder->ewma_value - encoder->ewma_previous_value;
    encoder->ewma_previous_value = encoder->ewma_value;
    
    
    // uint64_t mech_times_pp = (uint64_t)encoder->current_raw_value * (uint64_t)encoder->magnetic_pole_pairs;
    // uint32_t mech_elec_raw = (uint32_t)(mech_times_pp % ENC_MODULO);
    
    // uint32_t elec_raw;
    // if (mech_elec_raw >= encoder->electrical_offset) {
        //     elec_raw = mech_elec_raw - encoder->electrical_offset;
        // } else {
            //     elec_raw = mech_elec_raw + ENC_MODULO - encoder->electrical_offset;
            // }
            
            // encoder->electrical_angle_raw = elec_raw;
            // encoder->electrical_angle = (2.0f * 3.14159265359f * (float)elec_raw) / (float)ENC_MODULO;


    float mech_raw = fmodf(encoder->ewma_value, (float)ENC_MODULO);
    if (mech_raw < 0.0f) {
        mech_raw += (float)ENC_MODULO;
    }

    float mech_elec_raw = fmodf(
        mech_raw * (float)encoder->magnetic_pole_pairs,
        (float)ENC_MODULO
    );

    float elec_raw;
    if (mech_elec_raw >= (float)encoder->electrical_offset) {
        elec_raw = mech_elec_raw - (float)encoder->electrical_offset;
    } else {
        elec_raw = mech_elec_raw + (float)ENC_MODULO - (float)encoder->electrical_offset;
    }

    encoder->electrical_angle_raw = (uint32_t)elec_raw;
    encoder->electrical_angle =
        (2.0f * 3.14159265359f * elec_raw) / (float)ENC_MODULO;
}

void update_electrical_offset(encoder_t* encoder, uint32_t new_offset){
    if (!encoder) return;
    encoder->electrical_offset = new_offset;
}

double encoder_get_turns(const encoder_t* e) {
    return (double)e->position_ticks / (double)ENC_MODULO;
}


uint32_t mt6835_read_raw21(uint8_t *status_out)
{
    
    // Burst command (0xA) + start address 0x003
    // Sending 6 bytes: [cmd][addr][dummy][dummy][dummy][dummy]
    uint8_t tx[6] = { (uint8_t)(0xA << 4), 0x03, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rx[6] = {0};
    
    if (spi3_configure_for_encoder() != HAL_OK) {
        if (status_out) {
            *status_out = 0xFF;
        }
        return 0;
    }

    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET); // TAKE SPI
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, sizeof(tx), 100);
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET); // RELEASE SPI

    
    // Datasheet: 0x003 = ANGLE[20:13], 0x004 = ANGLE[12:5], 0x005 = ANGLE[4:0] + STATUS[2:0], 0x006 = CRC :contentReference[oaicite:1]{index=1}
    uint32_t raw =
        ((uint32_t)rx[2] << 13) |
        ((uint32_t)rx[3] << 5)  |
        ((uint32_t)rx[4] >> 3);

    raw &= 0x1FFFFF; // 21 bit

    if (status_out) {
        *status_out = (uint8_t)(rx[4] & 0x07); // STATUS[2:0]
    }

    return raw;
}

// angular velocity in radians per second
float get_angular_velocity(const encoder_t* encoder, float dt){
    if (!encoder || dt <= 0.0f) return 0.0f;

    float delta_ticks = encoder->ewma_delta;
    float velocity = (delta_ticks / dt) * ((2.0f * 3.14159265359f) / (float)ENC_MODULO);
    return velocity;
}
