/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f3xx_hal.h"
#include "string.h"
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>

#include <math.h>

#include "encoder.h"
#include "driver.h"
#include "foc.h"

#include "debug_utils.h"
#include "pi_controller.h"

#include "communication.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static drv8316_config_t g_drv_cfg = {
    .pwm_mode = DRV8316_PWM_MODE_6X,
    .slew = DRV8316_SLEW_50V_PER_US,
    .csa_gain = DRV8316_CSA_GAIN_0V60_PER_A,
    .sdo_push_pull = true,
    .enable_asr = false,
    .enable_aar = false,
    .clear_faults_on_init = true,
    .lock_registers_after_init = false,
    .verify_after_write = true,
};

static driver_current_offsets_t g_current_offsets = {0};
static driver_phase_currents_t g_phase_currents = {0};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENC_CS_GPIO_Port GPIOA
#define ENC_CS_Pin       GPIO_PIN_5
#define V_BUS_NOMINAL 20.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// READ SENSE ADC
volatile uint16_t ia_raw, ib_raw, ic_raw;
volatile float ia, ib, ic;

// Update encoder in timer interrupt (TIM6)
encoder_t encoder;
volatile float target_w = 0;
PIDController pid_w;




volatile bool foc_enabled = false;
PIController pi_d;
PIController pi_q;



float va, vb, vc;
volatile float id_ref = 0.0f;
volatile float iq_ref = 0.0f;
float id, iq;


// float aw = 0;
uint64_t prev_turns = 0;
float angular_velocity = 0;
volatile uint32_t adc_inj_cb_count = 0;


// float alpha_update = 0.8;
// float iq_ewma = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    if (target_w < -1000) return;
    // adc_inj_cb_count++;

    static uint8_t tick = 0;
    uint8_t skip = 1;
    if (++tick < skip) return;
    tick = 0;

    if (!foc_enabled) return;

    // const float dt_vel = 0.001778f;  // 4 × (1/2250 Hz)
    const float dt_vel = skip * 1.0f/1125.0f;  // 4 × (1/2250 Hz)

    float velocity_rpm = get_velocity_moving_average(&encoder) * 9.55741f;

//     float velocity_rpm = encoder.angular_velocity_ewma * 9.55741f;
    float error_w = target_w - velocity_rpm;

    // iq_ref = pid_update(&pid_w, error_w, dt_vel);
    float iq_cmd = pid_update(&pid_w, error_w, dt_vel);

    float max_step_up = 0.01f; // A per update
    float max_step_down = 0.5f; // A per update
    float diq = iq_cmd - iq_ref;
    if (diq >  max_step_up) diq = max_step_up;
    // if (diq < -max_step_down) diq = -max_step_down;
    iq_ref += diq;




    // float iq_ref_new = pid_update(&pid_w, error_w, dt_vel);
    // iq_ewma = alpha_update * iq_ref_new + (1 - alpha_update) * iq_ewma;
    // iq_ref_new = iq_ref;
    // iq_ref = iq_ewma;
}



void torque_control(float id_ref, float iq_ref, float theta_e){
  float i_alpha, i_beta;    
  clarke_transform(ia, ib, ic, &i_alpha, &i_beta);
  park_transform(i_alpha, i_beta, theta_e, &id, &iq);
  

  float err_d = id_ref - id;
  float err_q = iq_ref - iq;
  
  float vd = pi_update(&pi_d, err_d, 0.0002f);
  float vq = pi_update(&pi_q, err_q, 0.0002f); 
  
  float v_alpha, v_beta;
  inv_park_transform(vd, vq, theta_e, &v_alpha, &v_beta);
  inv_clarke_transform(v_alpha, v_beta, &va, &vb, &vc);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;
    // adc_inj_cb_count++;
    ic_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    ib_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    ia_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    driver_phase_currents_from_adc(&g_phase_currents,
        ia_raw,
        ib_raw,
        ic_raw,
        &g_current_offsets,
        g_drv_cfg.csa_gain);
        
        ia = g_phase_currents.a;
        ib = g_phase_currents.b;
        ic = g_phase_currents.c;
        // update_encoder(&encoder);
    if (foc_enabled){
            
      // FOC control loop
      
      update_encoder(&encoder);
      float theta_e = encoder.electrical_angle;
      torque_control(id_ref, iq_ref, theta_e);


      // SVPWM control
      float vmax = fmaxf(va, fmaxf(vb, vc));
      float vmin = fminf(va, fminf(vb, vc));
      
      float v0 = -0.5f * (vmax + vmin);

      float va_svpwm = va + v0;
      float vb_svpwm = vb + v0;
      float vc_svpwm = vc + v0;

      float du = 0.5f + va_svpwm / V_BUS_NOMINAL;
      float dv = 0.5f + vb_svpwm / V_BUS_NOMINAL;
      float dw = 0.5f + vc_svpwm / V_BUS_NOMINAL;
      // float du = 0.5f + va_svpwm;
      // float dv = 0.5f + vb_svpwm;
      // float dw = 0.5f + vc_svpwm;

      pwm_set(du, dv, dw);
      

      // SIN control
      // float du = 0.5f + 0.5f * va;
      // float dv = 0.5f + 0.5f * vb;
      // float dw = 0.5f + 0.5f * vc;

      // pwm_set(du, dv, dw);

    }

  }

  // ------- < CAN COMUNICATION > ----------- 
  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
      communication_rx_fifo0_pending_callback(hcan);
  }
  // ------- < / CAN COMUNICATION > ----------- 



  void calibrate_electrical_offset(encoder_t* encoder){
    if (!encoder) return;

    float v_alpha, v_beta;

    // align přes d-axis
    inv_park_transform(0.4f, 0.0f, 0.0f, &v_alpha, &v_beta);
    inv_clarke_transform(v_alpha, v_beta, &va, &vb, &vc);

    float du = 0.5f + va;
    float dv = 0.5f + vb;
    float dw = 0.5f + vc;
    pwm_set(du, dv, dw);

    HAL_Delay(800);

    uint32_t first = 0;
    int32_t acc = 0;
    int32_t prev = 0;

    for (int i = 0; i < 300; i++) {
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

    int32_t avg = acc / 300;
    while (avg < 0) avg += (int32_t)ENC_MODULO;
    while (avg >= (int32_t)ENC_MODULO) avg -= (int32_t)ENC_MODULO;

    uint32_t mech_raw_aligned = (uint32_t)avg;
    uint32_t mech_elec_raw =
        (uint32_t)(((uint64_t)mech_raw_aligned * encoder->magnetic_pole_pairs) % ENC_MODULO);

    update_electrical_offset(encoder, mech_elec_raw);

    pwm_set(0.5f, 0.5f, 0.5f);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  

  encoder = init_encoder(11, 0, true);
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
  }

  // float p = 0.11f;
  // float k = 5.088f;
  float p_i = 6.0f;
  float i_i = 22000.0f;
  float out_min_i = -20.5f;
  float out_max_i = 20.5f;
  pi_init(&pi_d, p_i, i_i, out_min_i, out_max_i); // id_ref regulator
  pi_init(&pi_q, p_i, i_i, out_min_i, out_max_i); // iq_ref regulator
  
  float p_w = 0.01f;
  float i_w = 0.8f;
  float d_w = 0.0f;
  float out_min_w = -0.0f;
  float out_max_w = 0.8f;
  pid_init(&pid_w, p_w, i_w, d_w, out_min_w, out_max_w); // iq_ref regulator

  
  char buffer[128];
  // 1) Start CH4 (CC4 events)
  // 2) Set period mid
  TIM1->CCR4 = TIM1->ARR / 2;
  
  pwm_init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  
  print("Booting...\r\n");
  
  drv8316_init(&g_drv_cfg);
  init_sin_table();

  // if (drv8316_init(&g_drv_cfg) != HAL_OK) {
  //   Error_Handler();
  // }
  
  // DEBUG: Read DRV8316 status registers
  drv8316_status_t drv_status = {0};
  if (drv8316_read_status(&drv_status) != HAL_OK) {
        print("DRV status read failed\r\n");
    } else {
          sprintf(buffer, "DRV ic=0x%02X st1=0x%02X st2=0x%02X\r\n",
            drv_status.ic_status,
            drv_status.status1,
            (uint8_t)(drv_status.status2 & 0x7F));
          print(buffer);
      }
      if (drv_status.ic_status & 0x01) print("DRV FAULT bit set\r\n");
      if (drv_status.ic_status & 0x02) print("DRV OT bit set\r\n");
      if (drv_status.ic_status & 0x04) print("DRV OVP bit set\r\n");
      if (drv_status.ic_status & 0x10) print("DRV OCP bit set\r\n");
      if (drv_status.ic_status & 0x20) print("DRV SPI fault bit set\r\n");
      if (drv_status.ic_status & 0x40) print("DRV buck fault bit set\r\n");
      
      
      
      
      if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
      }
      
      // Enable main output (TIM1 needs it)
      __HAL_TIM_MOE_ENABLE(&htim1);
      
      if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        Error_Handler();
      }
      
      /* motor bez buzení */
      pwm_set(0.5f, 0.5f, 0.5f);
      HAL_Delay(20);
      
      // Calibrate current sensor offsets
      if (driver_current_calibrate_offsets(&g_current_offsets,
        &ia_raw,
        &ib_raw,
        &ic_raw,
        256*4,
        1) != HAL_OK) {
          Error_Handler();
        }
        // Debug print current offsets
        // sprintf(buffer, "offs A=%u B=%u C=%u valid=%u\r\n",
        //   g_current_offsets.a,
        //   g_current_offsets.b,
        //   g_current_offsets.c,
        //   g_current_offsets.valid ? 1 : 0);
        // print(buffer);
        // spi3_configure_for_encoder();

        mt6835_init();
        calibrate_electrical_offset(&encoder);
        sprintf(buffer, "offset=%lu elec_angle=%f\r\n",
                (unsigned long)encoder.electrical_offset,
                encoder.electrical_angle);
        print(buffer);
        
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // open_loop();
    

  

  // ---- < Can communication > ----
  communication_init(&hcan);
  communication_start();
  can_message_t msg;
  // ---- < /Can communication > ----
  // init_sin_table();
  foc_enabled = true;

  print("Entering main loop\r\n");  
  // open_loop3(&encoder);
  static uint32_t last_ms = 0;
  static uint32_t last_count = 0;
  while (1){


    char buffer[128];
    // float velocity_rpm = encoder.angular_velocity_ewma * 9.55741f;
    float velocity_rpm = get_velocity_moving_average(&encoder) * 9.55741f;

    sprintf(buffer, "%f %f %f\r\n", velocity_rpm, target_w, iq_ref*1000.0f);
    print(buffer);

    // float err_w = target_w - angular_velocity;
    // iq_ref = pi_update(&pid_w, err_w, 0.002f);


    // float angular_velocity = -get_angular_velocity(&encoder, 0.002f) * 9.55741f;
    // float angular_velocity = encoder.angular_velocity * 9.55741f;
    
    // float angular_velocity = encoder.angular_velocity_ewma * 9.55741f;
    // sprintf(buffer, "%f\r\n", angular_velocity);
    // // sprintf(buffer, "%f\r\n", encoder.ewma_value / 2097152.0f);
    // print(buffer);


    // Timer frequency check
    // uint32_t now = HAL_GetTick();
    // if (now - last_ms >= 1000) {
    //     uint32_t cnt = adc_inj_cb_count;
    //     uint32_t diff = cnt - last_count;

    //     sprintf(buffer, "ADC inj cb freq: %lu Hz\r\n", (unsigned long)diff);
    //     print(buffer);

    //     last_count = cnt;
    //     last_ms = now;
    // }


    // CAN COMUNICATION 
    // if (communication_read(&msg))
    // {
    //   switch(msg.id){
    //     case SET_P:
    //       memcpy(&p_i, msg.data, sizeof(float));
    //       // sprintf(buffer, "Setting p to %f \r\n", p_w);
    //       // print(buffer);
    //       pi_init(&pi_d, p_i, i_i, out_min_i, out_max_i);
    //       pi_init(&pi_q, p_i, i_i, out_min_i, out_max_i);
    //       break;
    //     case SET_I:
    //       memcpy(&i_i, msg.data, sizeof(float));
    //       // sprintf(buffer, "Setting k to %f \r\n", i_w);
    //       // print(buffer);
    //       pi_init(&pi_d, p_i, i_i, out_min_i, out_max_i);
    //       pi_init(&pi_q, p_i, i_i, out_min_i, out_max_i);
    //       break;
    //     case SET_MIN:
    //       memcpy(&out_min_i, msg.data, sizeof(float));
    //       // sprintf(buffer, "Setting out_min to %f \r\n", out_min_w);
    //       // print(buffer);
    //       pi_init(&pi_d, p_i, i_i, out_min_i, out_max_i);
    //       pi_init(&pi_q, p_i, i_i, out_min_i, out_max_i);
    //       break;
    //     case SET_MAX:
    //       memcpy(&out_max_i, msg.data, sizeof(float));
    //       // sprintf(buffer, "Setting out_max to %f \r\n", out_max_w);
    //       // print(buffer);
    //       pi_init(&pi_d, p_i, i_i, out_min_i, out_max_i);
    //       pi_init(&pi_q, p_i, i_i, out_min_i, out_max_i);
    //       break;
    //     case SET_TARGET_VELOCITY:
    //       // float target_w;
    //       memcpy(&target_w, msg.data, sizeof(float));

    //       // sprintf(buffer, "Setting target_w to %f \r\n", target_w);
    //       // print(buffer);
    //       break;
    //     case SET_ID_REF:
    //       memcpy(&id_ref, msg.data, sizeof(float));
    //       // pi_reset(&pi_d);
    //       // pi_reset(&pi_q);
    //       // sprintf(buffer, "Setting id_ref to %f \r\n", id_ref);
    //       // print(buffer);
    //       break;
    //     case SET_IQ_REF:
    //       memcpy(&iq_ref, msg.data, sizeof(float));
    //       // pi_reset(&pi_d);
    //       // pi_reset(&pi_q);
    //       sprintf(buffer, "Setting iq_ref to %f \r\n", iq_ref);
    //       print(buffer);
    //       break;

    //     default:
    //       print("Unknown command");
    //   }
    // }

    if (communication_read(&msg))
    {
      switch(msg.id){
        case SET_P:
          memcpy(&p_w, msg.data, sizeof(float));
          // sprintf(buffer, "Setting p to %f \r\n", p_w);
          // print(buffer);
          pid_init(&pid_w, p_w, i_w, d_w, out_min_w, out_max_w);
          break;
        case SET_I:
          memcpy(&i_w, msg.data, sizeof(float));
          // sprintf(buffer, "Setting k to %f \r\n", i_w);
          // print(buffer);
          pid_init(&pid_w, p_w, i_w, d_w, out_min_w, out_max_w);
          break;
        case SET_D:
          memcpy(&d_w, msg.data, sizeof(float));
          pid_init(&pid_w, p_w, i_w, d_w, out_min_w, out_max_w);
          break;
        case SET_MIN:
          memcpy(&out_min_w, msg.data, sizeof(float));
          // sprintf(buffer, "Setting out_min to %f \r\n", out_min_w);
          // print(buffer);
          pid_init(&pid_w, p_w, i_w, d_w, out_min_w, out_max_w);
          break;
        case SET_MAX:
          memcpy(&out_max_w, msg.data, sizeof(float));
          // sprintf(buffer, "Setting out_max to %f \r\n", out_max_w);
          // print(buffer);
          pid_init(&pid_w, p_w, i_w, d_w, out_min_w, out_max_w);
          break;
        case SET_TARGET_VELOCITY:
          // float target_w;
          memcpy(&target_w, msg.data, sizeof(float));

          // sprintf(buffer, "Setting target_w to %f \r\n", target_w);
          // print(buffer);
          break;
        case SET_ID_REF:
          memcpy(&id_ref, msg.data, sizeof(float));
          // sprintf(buffer, "Setting id_ref to %f \r\n", id_ref);
          // print(buffer);
          break;
        case SET_IQ_REF:
          memcpy(&iq_ref, msg.data, sizeof(float));
          // sprintf(buffer, "Setting iq_ref to %f \r\n", iq_ref);
          // print(buffer);
          break;

        default:
          print("Unknown command");
      }
    }
  

  }
    
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 50;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31; // 31
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 250000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRIVER_CS_Pin|ENCODER_CS_Pin|IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DRIVER_CS_Pin ENCODER_CS_Pin IMU_CS_Pin */
  GPIO_InitStruct.Pin = DRIVER_CS_Pin|ENCODER_CS_Pin|IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */