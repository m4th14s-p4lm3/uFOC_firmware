// encoder.c
#include "encoder.h"
#include "stm32f3xx_hal.h"
#include <math.h>
#include "foc.h"
#include "driver.h"
#include "config.h"

#define ANGULAR_VELOCITY_EWMA_ALPHA 0.5f
#define ANGLE_VALUE_EWMA_ALPHA  0.8f
#define SAMPLE_DT 1.0f/8888.0f


#define ENC_CS_GPIO_Port GPIOA
#define ENC_CS_Pin       GPIO_PIN_5

extern SPI_HandleTypeDef hspi3;

/* -------------------------------------------------------------------------- */
/* CRC-8 pro MT6835                                                           */
/* -------------------------------------------------------------------------- */

#define MT6835_CRC_ENABLED  1

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


encoder_t init_encoder(uint8_t magnetic_pole_pairs, 
                        uint32_t electrical_offset, 
                        bool invert_dir){
    encoder_t encoder;
    uint8_t status_out;
    bool crc_ok = false;
    uint32_t new_raw_value = mt6835_read_raw21(&status_out, &crc_ok);
    
    encoder.invert_dir = invert_dir;

    encoder.current_raw_value = new_raw_value;
    encoder.prevous_raw_value = new_raw_value;
    

    encoder.angular_velocity = 0.0f;
    encoder.angular_velocity_ewma = 0.0f;
    encoder.angular_velocity_ewma_alpha = ANGULAR_VELOCITY_EWMA_ALPHA;


    encoder.ewma_previous_value = (float)new_raw_value;
    encoder.ewma_value = (float)new_raw_value;
    encoder.ewma_alpha = ANGLE_VALUE_EWMA_ALPHA;
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

    return encoder;
}


void update_encoder(encoder_t* encoder){
    if (!encoder) return;

    uint8_t status_out;
    bool crc_ok = false;
    uint32_t new_raw_value = mt6835_read_raw21(&status_out, &crc_ok);

    if (!crc_ok) {
        encoder->crc_error_count++;
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
    
    encoder->angular_velocity = get_angular_velocity_raw(encoder, SAMPLE_DT); // unit is ROTATIONS/SECOND - NOTE: Add "dt" property to init!
    encoder->angular_velocity_ewma = encoder->angular_velocity_ewma_alpha * encoder->angular_velocity + 
                                        (1 - encoder->angular_velocity_ewma_alpha) * encoder->angular_velocity_ewma;
    


    int64_t mech_raw_i = encoder->position_ticks % (int64_t)ENC_MODULO;
    if (mech_raw_i < 0) {
        mech_raw_i += (int64_t)ENC_MODULO;
    }


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
        (2.0f * M_PI * (float)elec_raw_i) / (float)ENC_MODULO;

    if (encoder->invert_dir){
        encoder->electrical_angle = -(encoder->electrical_angle - 2 * M_PI);
    }

}

void update_electrical_offset(encoder_t* encoder, 
                                uint32_t new_offset){
    encoder->electrical_offset = new_offset;
}

float encoder_get_turns(const encoder_t* encoder) {
    float sign = 1;
    if (encoder->invert_dir) sign = -1;
    return sign * (float)encoder->position_ticks / (float)ENC_MODULO;
}



void mt6835_init(){
    spi3_configure_for_encoder();
}


uint8_t tx[6] = { (uint8_t)(0xA << 4), 0x03, 0x00, 0x00, 0x00, 0x00 };
uint8_t rx[6] = {0};
uint32_t mt6835_read_raw21(uint8_t *status_out, bool *crc_ok)
{
    // Burst command (0xA) + start address 0x003
    // Frame: [cmd|addr][dummy x4] → rx: [don't care x2][ANGLE[20:13]][ANGLE[12:5]][ANGLE[4:0]+STATUS][CRC]

    if (crc_ok) {
        *crc_ok = false;
    }


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

// angular velocity in rotations per second
float get_angular_velocity_raw(const encoder_t* encoder, 
                                float dt){
    if (!encoder || dt <= 0.0f) return 0.0f;
    float sign = 1;
    if (encoder->invert_dir) sign = -1;


    float delta_ticks = encoder->ewma_delta;
    float velocity = (delta_ticks / dt) * (1.0f / (float)ENC_MODULO);
    return sign * velocity;
}

float get_angular_velocity(encoder_t* encoder){
    return encoder->angular_velocity_ewma;
}


static void DWT_Delay_ms(uint32_t ms)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
    }
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = ms * (SystemCoreClock / 1000U);
    while ((DWT->CYCCNT - start) < ticks);
}

extern ADC_HandleTypeDef hadc1;

void calibrate_electrical_offset(encoder_t* encoder){
    // THIS IS A BLOCKING FUNCTION 
    // YOU SHOULD SET CONTROL STATE TO NO_CONTROL BEFORE CALLING THIS FUNCITON!
    if (!encoder) return;
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOS);   // pause ADC ISR

    // DWT_Delay_ms(3);
    HAL_Delay(3);

    float v_alpha, v_beta;
    float va, vb, vc;

    inv_park_transform(5.0f, 0.0f, 0.0f, &v_alpha, &v_beta);
    inv_clarke_transform(v_alpha, v_beta, &va, &vb, &vc);

    float vmax = fmaxf(va, fmaxf(vb, vc));
    float vmin = fminf(va, fminf(vb, vc));
    float v0 = -0.5f * (vmax + vmin);
    

    float va_svpwm = va + v0;
    float vb_svpwm = vb + v0;
    float vc_svpwm = vc + v0;

    float du = 0.5f + va_svpwm / V_BUS_NOMINAL; // NOTE: REPLACE WITH V_BUS_NOMINAL!!!
    float dv = 0.5f + vb_svpwm / V_BUS_NOMINAL;
    float dw = 0.5f + vc_svpwm / V_BUS_NOMINAL;

    pwm_set(du, dv, dw);

    HAL_Delay(3);

    uint32_t first = 0;
    int32_t acc = 0;
    int32_t prev = 0;
    uint16_t num_samples = 300;
    for (int i = 0; i < num_samples; i++) {
        update_encoder(encoder);
        int32_t x = (int32_t)encoder->current_raw_value;

        if (i == 0) {
            first = (uint32_t)x;
            prev = x;
            acc = x;
        } else {
            int32_t dx = x - prev;
            if (dx > (int32_t)ENC_HALF_MODULO)  x -= (int32_t)ENC_MODULO;
            if (dx < -(int32_t)ENC_HALF_MODULO) x += (int32_t)ENC_MODULO;
            acc += x;
            prev = x;
        }

        HAL_Delay(2);
    }

    int32_t avg = acc / num_samples;
    while (avg < 0) avg += (int32_t)ENC_MODULO;
    while (avg >= (int32_t)ENC_MODULO) avg -= (int32_t)ENC_MODULO;

    uint32_t mech_raw_aligned = (uint32_t)avg;
    uint32_t mech_elec_raw =
        (uint32_t)(((uint64_t)mech_raw_aligned * encoder->magnetic_pole_pairs) % ENC_MODULO);

    update_electrical_offset(encoder, mech_elec_raw);

    pwm_set(0.5f, 0.5f, 0.5f);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS); // unpause ADC ISR

}