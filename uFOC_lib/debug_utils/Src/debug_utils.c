#include "string.h"
#include <stdio.h>
#include "stm32f3xx_hal.h"

#include "driver.h"

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

void print(const char *msg){
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}





