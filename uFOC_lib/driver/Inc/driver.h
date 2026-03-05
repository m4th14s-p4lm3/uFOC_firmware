#ifndef DRIVER_H
#define DRIVER_H


uint16_t read_adc_channel(ADC_HandleTypeDef *hadc, uint32_t channel);

void pwm_init();

void pwm_set(float du, float dv, float dw);

void open_loop(); // WILL BE REMOVED


#endif