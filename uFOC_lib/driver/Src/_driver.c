#include "driver.h"

#include "main.h"
#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define M_PI 3.14159265358979323846

extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi3;

/* -------------------------------------------------------------------------- */
/* DRV8316 register field definitions                                         */
/* -------------------------------------------------------------------------- */

#define DRV8316_CTRL1_REG_LOCK_UNLOCK      (0x03u)
#define DRV8316_CTRL1_REG_LOCK_LOCK        (0x06u)

#define DRV8316_CTRL2_CLR_FLT_MASK         (1u << 0)
#define DRV8316_CTRL2_PWM_MODE_MASK        (3u << 1)
#define DRV8316_CTRL2_SLEW_MASK            (3u << 3)
#define DRV8316_CTRL2_SDO_MODE_MASK        (1u << 5)

#define DRV8316_CTRL5_CSA_GAIN_MASK        (3u << 0)
#define DRV8316_CTRL5_EN_ASR_MASK          (1u << 2)
#define DRV8316_CTRL5_EN_AAR_MASK          (1u << 3)

#define DRV8316_CTRL6_BUCK_DIS_MASK        (1u << 0)
#define DRV8316_CTRL6_BUCK_SEL_MASK        (3u << 1)
#define DRV8316_CTRL6_BUCK_CL_MASK         (1u << 3)
#define DRV8316_CTRL6_BUCK_PS_DIS_MASK     (1u << 4)

#define DRV8316_CTRL6_BUCK_3V3             (0u << 1)

#define DRV8316_SPI_TIMEOUT_MS             (10u)
#define DRV8316_SPI_CS_GUARD_DELAY_US      (1u)

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static inline void drv8316_select(void)
{
    HAL_GPIO_WritePin(DRIVER_CS_GPIO_Port, DRIVER_CS_Pin, GPIO_PIN_RESET);
}

static inline void drv8316_deselect(void)
{
    HAL_GPIO_WritePin(DRIVER_CS_GPIO_Port, DRIVER_CS_Pin, GPIO_PIN_SET);
}

static inline uint8_t drv8316_even_parity_15b(uint16_t value_wo_parity)
{
    uint16_t x = value_wo_parity;
    x ^= (uint16_t)(x >> 8);
    x ^= (uint16_t)(x >> 4);
    x ^= (uint16_t)(x >> 2);
    x ^= (uint16_t)(x >> 1);
    return (uint8_t)(x & 0x01u);
}

static uint16_t drv8316_build_frame(bool read, uint8_t reg_addr, uint8_t data)
{
    uint16_t frame = 0u;
    frame |= (uint16_t)(read ? 1u : 0u) << 15;
    frame |= (uint16_t)(reg_addr & 0x3Fu) << 9;
    frame |= (uint16_t)data;
    frame |= (uint16_t)drv8316_even_parity_15b(frame) << 8;
    return frame;
}

static HAL_StatusTypeDef spi3_configure_for_drv8316(void)
{
    if (hspi3.Init.CLKPolarity == SPI_POLARITY_LOW &&
        hspi3.Init.CLKPhase == SPI_PHASE_2EDGE) {
        return HAL_OK;
    }

    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
    return HAL_SPI_Init(&hspi3);
}

static HAL_StatusTypeDef drv8316_spi_transfer_internal(uint16_t tx_data, uint16_t *rx_data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buf[2] = {
        (uint8_t)((tx_data >> 8) & 0xFFu),
        (uint8_t)(tx_data & 0xFFu)
    };
    uint8_t rx_buf[2] = {0u, 0u};

    if (rx_data == NULL) {
        return HAL_ERROR;
    }

    status = spi3_configure_for_drv8316();
    if (status != HAL_OK) {
        return status;
    }

    drv8316_select();
    for (volatile uint32_t i = 0u; i < (SystemCoreClock / 1000000u) * DRV8316_SPI_CS_GUARD_DELAY_US / 10u + 1u; ++i) {
        __NOP();
    }

    status = HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, 2u, DRV8316_SPI_TIMEOUT_MS);

    drv8316_deselect();
    HAL_Delay(2);

    *rx_data = (uint16_t)(((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1]);
    return status;
}

/* -------------------------------------------------------------------------- */
/* PWM                                                                        */
/* -------------------------------------------------------------------------- */

void pwm_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    TIM1->CCR4 = TIM1->ARR / 2;
}

void pwm_set(float du, float dv, float dw)
{
    const float duty_min = 0.0f;
    const float duty_max = 1.0f;
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    du = clampf(du, duty_min, duty_max);
    dv = clampf(dv, duty_min, duty_max);
    dw = clampf(dw, duty_min, duty_max);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(du * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dv * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dw * (float)arr));
}

void open_loop(void)
{
    float theta = 0.0f;
    const float ts = 0.0001f;
    const float fe = 400.0f;
    const float omega = 2.0f * (float)M_PI * fe;
    const float m = 0.80f;

    while (1) {
        theta += omega * ts;
        if (theta > 2.0f * (float)M_PI) {
            theta -= 2.0f * (float)M_PI;
        }

        const float su = sinf(theta);
        const float sv = sinf(theta - 2.0f * (float)M_PI / 3.0f);
        const float sw = sinf(theta - 4.0f * (float)M_PI / 3.0f);

        const float du = 0.5f + 0.5f * m * su;
        const float dv = 0.5f + 0.5f * m * sv;
        const float dw = 0.5f + 0.5f * m * sw;

        pwm_set(du, dv, dw);

        for (volatile int i = 0; i < (int)(SystemCoreClock * ts / 10.0f); ++i) {
            __NOP();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* SPI low-level                                                              */
/* -------------------------------------------------------------------------- */

uint16_t drv8316_spi_transfer(uint16_t tx_data)
{
    uint16_t rx_data = 0u;
    (void)drv8316_spi_transfer_internal(tx_data, &rx_data);
    return rx_data;
}

HAL_StatusTypeDef drv8316_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint16_t dummy_rx = 0u;
    return drv8316_spi_transfer_internal(drv8316_build_frame(false, reg_addr, data), &dummy_rx);
}

HAL_StatusTypeDef drv8316_read_reg(uint8_t reg_addr, uint8_t *data)
{
    uint16_t rx_word = 0u;
    HAL_StatusTypeDef status;

    if (data == NULL) {
        return HAL_ERROR;
    }

    status = drv8316_spi_transfer_internal(drv8316_build_frame(true, reg_addr, 0x00u), &rx_word);
    if (status != HAL_OK) {
        return status;
    }

    *data = (uint8_t)(rx_word & 0xFFu);
    return HAL_OK;
}

HAL_StatusTypeDef drv8316_update_reg(uint8_t reg_addr, uint8_t mask, uint8_t value)
{
    uint8_t reg_val = 0u;

    if (drv8316_read_reg(reg_addr, &reg_val) != HAL_OK) {
        return HAL_ERROR;
    }

    reg_val = (uint8_t)((reg_val & (uint8_t)(~mask)) | (value & mask));
    return drv8316_write_reg(reg_addr, reg_val);
}

HAL_StatusTypeDef drv8316_unlock_registers(void)
{
    return drv8316_write_reg(DRV8316_REG_CTRL1, DRV8316_CTRL1_REG_LOCK_UNLOCK);
}

HAL_StatusTypeDef drv8316_lock_registers(void)
{
    return drv8316_write_reg(DRV8316_REG_CTRL1, DRV8316_CTRL1_REG_LOCK_LOCK);
}

HAL_StatusTypeDef drv8316_clear_faults(void)
{
    return drv8316_update_reg(DRV8316_REG_CTRL2,
                              DRV8316_CTRL2_CLR_FLT_MASK,
                              DRV8316_CTRL2_CLR_FLT_MASK);
}

/* -------------------------------------------------------------------------- */
/* DRV8316 config                                                             */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef drv8316_init(const drv8316_config_t *cfg)
{
    uint8_t ctrl2 = 0u;
    uint8_t ctrl5 = 0u;
    uint8_t ctrl6 = 0u;
    uint8_t rb = 0u;

    if (cfg == NULL) {
        return HAL_ERROR;
    }

    drv8316_deselect();
    HAL_Delay(2);

    if (drv8316_unlock_registers() != HAL_OK) {
        return HAL_ERROR;
    }

    ctrl2 = 0u;
    ctrl2 |= cfg->sdo_push_pull ? DRV8316_CTRL2_SDO_MODE_MASK : 0u;
    ctrl2 |= (uint8_t)((((uint8_t)cfg->slew) << 3) & DRV8316_CTRL2_SLEW_MASK);
    ctrl2 |= (uint8_t)((((uint8_t)cfg->pwm_mode) << 1) & DRV8316_CTRL2_PWM_MODE_MASK);
    ctrl2 |= cfg->clear_faults_on_init ? DRV8316_CTRL2_CLR_FLT_MASK : 0u;

    ctrl5 = 0u;
    ctrl5 |= cfg->enable_aar ? DRV8316_CTRL5_EN_AAR_MASK : 0u;
    ctrl5 |= cfg->enable_asr ? DRV8316_CTRL5_EN_ASR_MASK : 0u;
    ctrl5 |= (uint8_t)(((uint8_t)cfg->csa_gain) & DRV8316_CTRL5_CSA_GAIN_MASK);

    ctrl6 = 0u;
    ctrl6 |= DRV8316_CTRL6_BUCK_3V3;
    ctrl6 |= DRV8316_CTRL6_BUCK_PS_DIS_MASK;

    if (drv8316_write_reg(DRV8316_REG_CTRL2, ctrl2) != HAL_OK) {
        return HAL_ERROR;
    }

    if (drv8316_write_reg(DRV8316_REG_CTRL5, ctrl5) != HAL_OK) {
        return HAL_ERROR;
    }

    if (drv8316_write_reg(DRV8316_REG_CTRL6, ctrl6) != HAL_OK) {
        return HAL_ERROR;
    }

    if (cfg->clear_faults_on_init) {
        if (drv8316_clear_faults() != HAL_OK) {
            return HAL_ERROR;
        }
    }

    if (cfg->verify_after_write) {
        if (drv8316_read_reg(DRV8316_REG_CTRL2, &rb) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((rb & (DRV8316_CTRL2_SDO_MODE_MASK |
                   DRV8316_CTRL2_SLEW_MASK |
                   DRV8316_CTRL2_PWM_MODE_MASK)) !=
            (ctrl2 & (DRV8316_CTRL2_SDO_MODE_MASK |
                      DRV8316_CTRL2_SLEW_MASK |
                      DRV8316_CTRL2_PWM_MODE_MASK))) {
            return HAL_ERROR;
        }

        if (drv8316_read_reg(DRV8316_REG_CTRL5, &rb) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((rb & (DRV8316_CTRL5_EN_AAR_MASK |
                   DRV8316_CTRL5_EN_ASR_MASK |
                   DRV8316_CTRL5_CSA_GAIN_MASK)) !=
            (ctrl5 & (DRV8316_CTRL5_EN_AAR_MASK |
                      DRV8316_CTRL5_EN_ASR_MASK |
                      DRV8316_CTRL5_CSA_GAIN_MASK))) {
            return HAL_ERROR;
        }

        if (drv8316_read_reg(DRV8316_REG_CTRL6, &rb) != HAL_OK) {
            return HAL_ERROR;
        }
        if ((rb & (DRV8316_CTRL6_BUCK_DIS_MASK |
                   DRV8316_CTRL6_BUCK_SEL_MASK |
                   DRV8316_CTRL6_BUCK_CL_MASK |
                   DRV8316_CTRL6_BUCK_PS_DIS_MASK)) !=
            (ctrl6 & (DRV8316_CTRL6_BUCK_DIS_MASK |
                      DRV8316_CTRL6_BUCK_SEL_MASK |
                      DRV8316_CTRL6_BUCK_CL_MASK |
                      DRV8316_CTRL6_BUCK_PS_DIS_MASK))) {
            return HAL_ERROR;
        }
    }

    if (cfg->lock_registers_after_init) {
        if (drv8316_lock_registers() != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Current sense                                                              */
/* -------------------------------------------------------------------------- */

float drv8316_gain_to_v_per_a(drv8316_csa_gain_t gain)
{
    switch (gain) {
    case DRV8316_CSA_GAIN_0V15_PER_A:
        return 0.15f;
    case DRV8316_CSA_GAIN_0V30_PER_A:
        return 0.30f;
    case DRV8316_CSA_GAIN_0V60_PER_A:
        return 0.60f;
    case DRV8316_CSA_GAIN_1V20_PER_A:
        return 1.20f;
    default:
        return 0.15f;
    }
}

float driver_adc_to_current(uint16_t adc_raw,
                            uint16_t offset_raw,
                            drv8316_csa_gain_t gain)
{
    const float gain_v_per_a = drv8316_gain_to_v_per_a(gain);
    const float adc_lsb_volts = DRIVER_ADC_VREF_VOLTS / DRIVER_ADC_MAX_COUNTS;
    const float delta_counts = (float)((int32_t)adc_raw - (int32_t)offset_raw);
    const float delta_volts = delta_counts * adc_lsb_volts;

    return delta_volts / gain_v_per_a;
}

void driver_phase_currents_from_adc(driver_phase_currents_t *currents,
                                    uint16_t ia_raw,
                                    uint16_t ib_raw,
                                    uint16_t ic_raw,
                                    const driver_current_offsets_t *offsets,
                                    drv8316_csa_gain_t gain)
{
    if ((currents == NULL) || (offsets == NULL) || (!offsets->valid)) {
        return;
    }

    currents->a = driver_adc_to_current(ia_raw, offsets->a, gain);
    currents->b = driver_adc_to_current(ib_raw, offsets->b, gain);
    currents->c = driver_adc_to_current(ic_raw, offsets->c, gain);
}

HAL_StatusTypeDef driver_current_calibrate_offsets(driver_current_offsets_t *offsets,
                                                   volatile const uint16_t *ia_raw,
                                                   volatile const uint16_t *ib_raw,
                                                   volatile const uint16_t *ic_raw,
                                                   uint16_t sample_count,
                                                   uint32_t inter_sample_delay_ms)
{
    uint32_t sum_a = 0u;
    uint32_t sum_b = 0u;
    uint32_t sum_c = 0u;

    if ((offsets == NULL) ||
        (ia_raw == NULL) ||
        (ib_raw == NULL) ||
        (ic_raw == NULL) ||
        (sample_count == 0u)) {
        return HAL_ERROR;
    }

    offsets->a = 0u;
    offsets->b = 0u;
    offsets->c = 0u;
    offsets->valid = false;

    for (uint16_t i = 0u; i < sample_count; ++i) {
        sum_a += *ia_raw;
        sum_b += *ib_raw;
        sum_c += *ic_raw;

        if (inter_sample_delay_ms > 0u) {
            HAL_Delay(inter_sample_delay_ms);
        }
    }

    offsets->a = (uint16_t)(sum_a / sample_count);
    offsets->b = (uint16_t)(sum_b / sample_count);
    offsets->c = (uint16_t)(sum_c / sample_count);
    offsets->valid = true;

    return HAL_OK;
}

HAL_StatusTypeDef drv8316_read_status(drv8316_status_t *st)
{
    if (st == NULL) {
        return HAL_ERROR;
    }

    if (drv8316_read_reg(DRV8316_REG_IC_STATUS, &st->ic_status) != HAL_OK) {
        return HAL_ERROR;
    }
    if (drv8316_read_reg(DRV8316_REG_STATUS1, &st->status1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (drv8316_read_reg(DRV8316_REG_STATUS2, &st->status2) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}