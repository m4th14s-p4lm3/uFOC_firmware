#ifndef UFOC_DRIVER_H
#define UFOC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


typedef struct
{
    uint8_t ic_status;
    uint8_t status1;
    uint8_t status2;
} drv8316_status_t;

HAL_StatusTypeDef drv8316_read_status(drv8316_status_t *st);


/* -------------------------------------------------------------------------- */
/* ADC conversion constants                                                   */
/* -------------------------------------------------------------------------- */

#define DRIVER_ADC_VREF_VOLTS      (3.3f)
#define DRIVER_ADC_MAX_COUNTS      (4095.0f)

/* -------------------------------------------------------------------------- */
/* DRV8316 register map                                                       */
/* -------------------------------------------------------------------------- */

#define DRV8316_REG_IC_STATUS      (0x00u)
#define DRV8316_REG_STATUS_1       (0x01u)
#define DRV8316_REG_STATUS_2       (0x02u)
#define DRV8316_REG_CTRL1          (0x03u)
#define DRV8316_REG_CTRL2          (0x04u)
#define DRV8316_REG_CTRL3          (0x05u)
#define DRV8316_REG_CTRL4          (0x06u)
#define DRV8316_REG_CTRL5          (0x07u)
#define DRV8316_REG_CTRL6          (0x08u)

/* -------------------------------------------------------------------------- */
/* Enums                                                                      */
/* -------------------------------------------------------------------------- */

#define DRV8316_REG_IC_STATUS   (0x00u)
#define DRV8316_REG_STATUS1     (0x01u)
#define DRV8316_REG_STATUS2     (0x02u)
#define DRV8316_REG_CTRL1       (0x03u)
#define DRV8316_REG_CTRL2       (0x04u)
#define DRV8316_REG_CTRL3       (0x05u)
#define DRV8316_REG_CTRL4       (0x06u)
#define DRV8316_REG_CTRL5       (0x07u)
#define DRV8316_REG_CTRL6       (0x08u)

typedef enum
{
    DRV8316_PWM_MODE_6X = 0u,
    DRV8316_PWM_MODE_6X_CURRENT_LIMIT = 1u,
    DRV8316_PWM_MODE_3X = 2u,
    DRV8316_PWM_MODE_3X_CURRENT_LIMIT = 3u
} drv8316_pwm_mode_t;

typedef enum
{
    DRV8316_SLEW_25V_PER_US = 0u,
    DRV8316_SLEW_50V_PER_US = 1u,
    DRV8316_SLEW_125V_PER_US = 2u,
    DRV8316_SLEW_200V_PER_US = 3u
} drv8316_slew_t;

typedef enum
{
    DRV8316_CSA_GAIN_0V15_PER_A = 0u,
    DRV8316_CSA_GAIN_0V30_PER_A = 1u,
    DRV8316_CSA_GAIN_0V60_PER_A = 2u,
    DRV8316_CSA_GAIN_1V20_PER_A = 3u
} drv8316_csa_gain_t;

/* -------------------------------------------------------------------------- */
/* Config / data structs                                                      */
/* -------------------------------------------------------------------------- */

typedef struct
{
    drv8316_pwm_mode_t pwm_mode;
    drv8316_slew_t slew;
    drv8316_csa_gain_t csa_gain;
    bool sdo_push_pull;
    bool enable_asr;
    bool enable_aar;
    bool clear_faults_on_init;
    bool lock_registers_after_init;
    bool verify_after_write;
} drv8316_config_t;

typedef struct
{
    uint16_t a;
    uint16_t b;
    uint16_t c;
    bool valid;
} driver_current_offsets_t;

typedef struct
{
    float a;
    float b;
    float c;
} driver_phase_currents_t;

/* -------------------------------------------------------------------------- */
/* PWM                                                                        */
/* -------------------------------------------------------------------------- */

void pwm_init(void);
void pwm_set(float du, float dv, float dw);
void open_loop(void);

/* -------------------------------------------------------------------------- */
/* DRV8316 SPI                                                                */
/* -------------------------------------------------------------------------- */

uint16_t drv8316_spi_transfer(uint16_t tx_data);
HAL_StatusTypeDef drv8316_write_reg(uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef drv8316_read_reg(uint8_t reg_addr, uint8_t *data);
HAL_StatusTypeDef drv8316_update_reg(uint8_t reg_addr, uint8_t mask, uint8_t value);

HAL_StatusTypeDef drv8316_unlock_registers(void);
HAL_StatusTypeDef drv8316_lock_registers(void);
HAL_StatusTypeDef drv8316_clear_faults(void);
HAL_StatusTypeDef drv8316_init(const drv8316_config_t *cfg);

/* -------------------------------------------------------------------------- */
/* Current sense                                                              */
/* -------------------------------------------------------------------------- */

float drv8316_gain_to_v_per_a(drv8316_csa_gain_t gain);
float driver_adc_to_current(uint16_t adc_raw,
                            uint16_t offset_raw,
                            drv8316_csa_gain_t gain);

void driver_phase_currents_from_adc(driver_phase_currents_t *currents,
                                    uint16_t ia_raw,
                                    uint16_t ib_raw,
                                    uint16_t ic_raw,
                                    const driver_current_offsets_t *offsets,
                                    drv8316_csa_gain_t gain);

HAL_StatusTypeDef driver_current_calibrate_offsets(driver_current_offsets_t *offsets,
                                                   volatile const uint16_t *ia_raw,
                                                   volatile const uint16_t *ib_raw,
                                                   volatile const uint16_t *ic_raw,
                                                   uint16_t sample_count,
                                                   uint32_t inter_sample_delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* UFOC_DRIVER_H */