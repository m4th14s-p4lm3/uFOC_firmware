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




uint16_t drv8316_spi_transfer(uint16_t tx_data);
void drv8316_write_reg(uint8_t reg_addr, uint8_t data);
uint8_t drv8316_read_reg(uint8_t reg_addr);
void drv8316_init(void);



#endif