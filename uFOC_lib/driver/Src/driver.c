#include "stm32f3xx_hal.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern  ADC_HandleTypeDef hadc1;


uint16_t read_adc_channel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
    
    HAL_ADC_ConfigChannel(hadc, &sConfig);
    
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint16_t v = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    
    return v;
}

void pwm_init(){
    // Start PWM outputs
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // Start complementary outputs (CHxN)
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    TIM1->CCR4 = TIM1->ARR / 2; 
}

void pwm_set(float du, float dv, float dw)
{
  uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(&htim1);

  if (du < 0) du = 0; if (du > 1) du = 1;
  if (dv < 0) dv = 0; if (dv > 1) dv = 1;
  if (dw < 0) dw = 0; if (dw > 1) dw = 1;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(du * ARR)); // INHC
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * ARR)); // INHB
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dw * ARR)); // INHA
}

void open_loop(){
  float theta = 0.0f;

  // open-loop parametry
  const float Ts = 0.0001f;
  const float fe = 360.0f;            // frekvence [Hz] 
  const float omega = 2.0f * (float)M_PI * fe;
  const float m = 0.80f;             // modulace (0..~0.9),

  while (1)
  {
    theta += omega * Ts;
    if (theta > 2.0f * (float)M_PI) theta -= 2.0f * (float)M_PI;

    float su = sinf(theta);
    float sv = sinf(theta - 2.0f*(float)M_PI/3.0f);
    float sw = sinf(theta - 4.0f*(float)M_PI/3.0f);

    // map -1..1 -> 0..1 s modulací m
    float du = 0.5f + 0.5f * m * su;
    float dv = 0.5f + 0.5f * m * sv;
    float dw = 0.5f + 0.5f * m * sw;

    pwm_set(du, dv, dw);

    for (volatile int i=0; i< (int)(SystemCoreClock * Ts / 10); i++) { __NOP(); }
  }
}
