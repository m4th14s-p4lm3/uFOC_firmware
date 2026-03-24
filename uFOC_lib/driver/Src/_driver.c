#include "stm32f3xx_hal.h"
#include <math.h>

#define DRV8316_REG_CTRL2 0x04
#define DRV8316_REG_CTRL5 0x07

extern TIM_HandleTypeDef htim1;
extern  ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi3;


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

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void pwm_set(float du, float dv, float dw)
{
    const float duty_min = 0.02f;
    const float duty_max = 0.98f;
    uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(&htim1);

    du = clampf(du, duty_min, duty_max);
    dv = clampf(dv, duty_min, duty_max);
    dw = clampf(dw, duty_min, duty_max);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(du * ARR)); // A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * ARR)); // B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dw * ARR)); // C
}

void open_loop(){
  float theta = 0.0f;

  // open-loop parametry
  const float Ts = 0.0001f;
  const float fe = 400.0f;            // frekvence [Hz] 
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



