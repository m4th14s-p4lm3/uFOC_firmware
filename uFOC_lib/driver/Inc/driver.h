#ifndef UFOC_DRIVER_H
#define UFOC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define DRIVER_ADC_VREF_VOLTS      (3.3f)
#define DRIVER_ADC_MAX_COUNTS      (4095.0f)

typedef enum
{
    DRV8316_CSA_GAIN_0V15_PER_A = 0u,
    DRV8316_CSA_GAIN_0V30_PER_A = 1u,
    DRV8316_CSA_GAIN_0V60_PER_A = 2u,
    DRV8316_CSA_GAIN_1V20_PER_A = 3u
} drv8316_csa_gain_t;

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

void pwm_init(void);
void pwm_set(float du, float dv, float dw);

HAL_StatusTypeDef drv8316_init(drv8316_csa_gain_t csa_gain);

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
