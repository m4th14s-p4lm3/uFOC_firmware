#include "string.h"
#include <stdio.h>
#include "stm32f3xx_hal.h"

#include "driver.h"

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

void print(const char *msg){
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void print_adc_input(){
    uint16_t sense_a = read_adc_channel(&hadc1, ADC_CHANNEL_3); // PA2
    uint16_t sense_b = read_adc_channel(&hadc1, ADC_CHANNEL_2); // PA1
    uint16_t sense_c = read_adc_channel(&hadc1, ADC_CHANNEL_1); // PA0
    char buffer[32];
    sprintf(buffer, "%d %d %d \r\n", sense_a, sense_b, sense_c);
    // sprintf(buffer, "%.5f\r\n", rotations);
    print(buffer); 
}




