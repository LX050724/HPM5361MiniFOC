#pragma once
#include "hpm_adc.h"
#include "hpm_interrupt.h"
#include "hpm_soc.h"
#include "project_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*adc_callback_t)(ADC16_Type *, uint32_t);

void adc_init(uint32_t sample_cycle);

void adc_set_callback(adc_callback_t cb);
uint16_t adc_GetRaw_V();
uint16_t adc_GetRaw_W();
uint16_t adc_GetRaw_VBUS();

static inline float adc_voltage(int raw)
{
    return (raw >> ADC_IGNORE_BIT) * 3.3f / (65536 >> ADC_IGNORE_BIT);
}

static inline void adc_enable_irq(uint32_t pri)
{
    intc_m_enable_irq_with_priority(IRQn_ADC0, pri);
    intc_m_enable_irq_with_priority(IRQn_ADC1, pri);
}

static inline void adc_disable_irq()
{
    intc_m_disable_irq(IRQn_ADC0);
    intc_m_disable_irq(IRQn_ADC1);
}

static inline void adc_enable_it()
{
    adc_type adc = {.module = adc_module_adc16};
    adc.adc_base.adc16 = HPM_ADC0;
    hpm_adc_enable_interrupts(&adc, adc16_event_trig_complete);
    adc.adc_base.adc16 = HPM_ADC1;
    hpm_adc_enable_interrupts(&adc, adc16_event_trig_complete);
}

static inline void adc_disable_it()
{
    adc_type adc = {.module = adc_module_adc16};
    adc.adc_base.adc16 = HPM_ADC0;
    hpm_adc_disable_interrupts(&adc, adc16_event_trig_complete);
    adc.adc_base.adc16 = HPM_ADC1;
    hpm_adc_disable_interrupts(&adc, adc16_event_trig_complete);
}

#ifdef __cplusplus
}
#endif