#pragma once

#include "foc/foc_core.h"
#include "hardware/adc_init.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int adc_calibration_w;
    int adc_calibration_v;
    int last_adc_raw_v;
    int last_adc_raw_w;
} CurrentCal_t;

static inline void current_get_cal(CurrentCal_t *self, uint16_t raw_v, uint16_t raw_w, foc_uvw_current_t *cur)
{
#if ADC_ENABLE_FILTER == 1
    cur->iv = adc_voltage(((raw_v + self->last_adc_raw_v) >> 1) - self->adc_calibration_v) / -CURRENT_COE;
    cur->iw = adc_voltage(((raw_w + self->last_adc_raw_w) >> 1) - self->adc_calibration_w) / -CURRENT_COE;
    self->last_adc_raw_v = raw_v;
    self->last_adc_raw_w = raw_w;
#else
    cur->iv = adc_voltage(raw_v - self->adc_calibration_v) / -CURRENT_COE;
    cur->iw = adc_voltage(raw_w - self->adc_calibration_w) / -CURRENT_COE;
#endif
    cur->iu = 0 - cur->iv - cur->iw;
}

static inline void current_set_calibration(CurrentCal_t *self, int32_t v_value, int32_t w_value)
{
    self->adc_calibration_v = self->last_adc_raw_v = v_value;
    self->adc_calibration_w = self->last_adc_raw_w = v_value;
}

static inline void current_init(CurrentCal_t *self)
{
    current_set_calibration(self, (65536 >> ADC_IGNORE_BIT) / 2, (65536 >> ADC_IGNORE_BIT) / 2);
}

#ifdef __cplusplus
}
#endif