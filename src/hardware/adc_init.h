#pragma once
#include "hpm_adc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_ATTR ATTR_PLACE_AT_NONCACHEABLE ATTR_ALIGN(ADC_SOC_DMA_ADDR_ALIGNMENT)

typedef struct ATTR_ALIGN(ADC_SOC_DMA_ADDR_ALIGNMENT)
{
    adc_type adc_w;
    adc_type adc_v;
    volatile struct {
        uint32_t value : 16;
        uint32_t : 4;
        uint32_t channel : 4;
        uint32_t source : 5;
        uint32_t : 2;
        uint32_t flag : 1;
    } adc_w_buff[48], adc_v_buff[48];
} CurrentADC_t;

void adc_init(CurrentADC_t *self, uint32_t sample_cycle, void (*isr_callback)(ADC16_Type *, uint32_t));

#ifdef __cplusplus
}
#endif