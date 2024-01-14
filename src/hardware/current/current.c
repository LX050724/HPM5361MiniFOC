#include "current.h"
#include "hardware/adc_init.h"
#include <stdint.h>

typedef struct
{
    int adc_calibration_w;
    int adc_calibration_v;
    int last_adc_raw_v;
    int last_adc_raw_w;
} CurrentCal_t;

static CurrentCal_t current;

void current_init()
{
    current_set_calibration((65536 >> ADC_IGNORE_BIT) / 2, (65536 >> ADC_IGNORE_BIT) / 2);
}

void current_get_cal(uint16_t raw_v, uint16_t raw_w, foc_uvw_current_t *cur)
{
#if ADC_ENABLE_FILTER == 1
    cur->iv = adc_voltage(((raw_v + current.last_adc_raw_v) >> 1) - current.adc_calibration_v) / -CURRENT_COE;
    cur->iw = adc_voltage(((raw_w + current.last_adc_raw_w) >> 1) - current.adc_calibration_w) / -CURRENT_COE;
    current.last_adc_raw_v = raw_v;
    current.last_adc_raw_w = raw_w;
#else
    cur->iv = adc_voltage(raw_v - current.adc_calibration_v) / -CURRENT_COE;
    cur->iw = adc_voltage(raw_w - current.adc_calibration_w) / -CURRENT_COE;
#endif
    cur->iu = 0 - cur->iv - cur->iw;
}

void current_set_calibration(int32_t v_value, int32_t w_value)
{
    current.adc_calibration_v = current.last_adc_raw_v = v_value;
    current.adc_calibration_w = current.last_adc_raw_w = v_value;
}
