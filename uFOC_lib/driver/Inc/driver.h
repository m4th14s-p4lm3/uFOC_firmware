#ifndef DRIVER_H
#define DRIVER_H

#define ADC_MAX 4095.0f // 2^12 - we are using 12 bit adc
#define ADC_VREF 3.3f
#define CURRENT_GAIN 20.0f   // příklad
// #define ADC_TO_CURRENT (ADC_VREF / (ADC_MAX * CURRENT_GAIN))
#define ADC_TO_CURRENT (1.0f / 2048.0f)


uint16_t read_adc_channel(ADC_HandleTypeDef *hadc, uint32_t channel);

void pwm_init();

void pwm_set(float du, float dv, float dw);

void open_loop(); // WILL BE REMOVED


#endif