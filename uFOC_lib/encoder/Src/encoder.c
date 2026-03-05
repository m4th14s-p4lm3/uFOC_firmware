// encoder.c
#include "encoder.h"
#include "stm32f3xx_hal.h"

#define ENC_CS_GPIO_Port GPIOA
#define ENC_CS_Pin       GPIO_PIN_5

#define MAX_ENCODER_RAW_VALUE 2097151u
#define ENC_MODULO            (MAX_ENCODER_RAW_VALUE + 1u)   // 2097152
#define ENC_HALF_MODULO       (ENC_MODULO / 2u)              // 1048576


extern SPI_HandleTypeDef hspi3;

encoder_t init_encoder(){
    encoder_t encoder;
    uint8_t status_out;
    uint32_t new_raw_value = mt6835_read_raw21(&status_out);
    
    encoder.current_raw_value = new_raw_value;
    encoder.prevous_raw_value = new_raw_value;
    encoder.position_ticks = new_raw_value;
    encoder.last_delta = 0;
    encoder.valid = 1;
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
}

double encoder_get_turns(const encoder_t* e) {
    return (double)e->position_ticks / (double)ENC_MODULO - 1;
}


uint32_t mt6835_read_raw21(uint8_t *status_out)
{
    // Burst command (0xA) + start address 0x003
    // Sending 6 bytes: [cmd][addr][dummy][dummy][dummy][dummy]
    uint8_t tx[6] = { (uint8_t)(0xA << 4), 0x03, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rx[6] = {0};

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
