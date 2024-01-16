
#include "adc_init.h"
#include "board.h"
#include "hpm_adc.h"
#include <stdint.h>

static adc_callback_t __isr_callback;

typedef struct ATTR_ALIGN(ADC_SOC_DMA_ADDR_ALIGNMENT)
{
    adc_type adc_w;
    adc_type adc_v;
    volatile struct
    {
        uint32_t value : 16;
        uint32_t : 4;
        uint32_t channel : 4;
        uint32_t source : 5;
        uint32_t : 2;
        uint32_t flag : 1;
    } adc_w_buff[48], adc_v_buff[48];
} CurrentADC_t;

static DMA_ATTR CurrentADC_t adc;

void init_trigger_cfg(void)
{
    adc_pmt_config_t pmt_cfg2 = {}, pmt_cfg1 = {};

    pmt_cfg1.module = ADCX_MODULE_ADC16;
    pmt_cfg1.config.adc16.trig_len = 1;
    pmt_cfg1.config.adc16.inten[0] = true;
    pmt_cfg1.config.adc16.trig_ch = BOARD_BLDC_ADC_CH_W_TRG;
    pmt_cfg1.config.adc16.adc_ch[0] = BOARD_BLDC_ADC_CH_U;
    pmt_cfg1.adc_base.adc16 = adc.adc_w.adc_base.adc16;
    hpm_adc_set_preempt_config(&pmt_cfg1);
    adc16_set_pmt_queue_enable(adc.adc_w.adc_base.adc16, BOARD_BLDC_ADC_CH_W_TRG, true);

    pmt_cfg2.module = ADCX_MODULE_ADC16;
    pmt_cfg2.config.adc16.trig_len = 2;
    pmt_cfg2.config.adc16.inten[1] = true;
    pmt_cfg2.config.adc16.trig_ch = BOARD_BLDC_ADC_CH_V_TRG;
    pmt_cfg2.config.adc16.adc_ch[0] = BOARD_BLDC_ADC_CH_V;
    pmt_cfg2.config.adc16.adc_ch[1] = BOARD_BLDC_ADC_CH_VBUS_1;
    pmt_cfg2.adc_base.adc16 = adc.adc_v.adc_base.adc16;
    hpm_adc_set_preempt_config(&pmt_cfg2);
    adc16_set_pmt_queue_enable(adc.adc_v.adc_base.adc16, BOARD_BLDC_ADC_CH_V_TRG, true);
}

void adc_init(uint32_t sample_cycle)
{
    adc_config_t cfg;
    adc_channel_config_t ch_cfg;
    cfg.module = ADCX_MODULE_ADC16;

    adc.adc_w.adc_base.adc16 = BOARD_BLDC_ADC_U_BASE;
    adc.adc_w.module = adc_module_adc16;
    adc.adc_v.adc_base.adc16 = BOARD_BLDC_ADC_V_BASE;
    adc.adc_v.module = adc_module_adc16;

    board_init_adc16_pins();
    board_init_adc16_clock(HPM_ADC0, true);
    board_init_adc16_clock(HPM_ADC1, true);
    hpm_adc_init_default_config(&cfg);

    cfg.config.adc16.res = adc16_res_16_bits;
    cfg.config.adc16.conv_mode = adc16_conv_mode_preemption;
    cfg.config.adc16.adc_clk_div = adc16_clock_divider_4;
    cfg.config.adc16.sel_sync_ahb = false;
    cfg.config.adc16.adc_ahb_en = true;

    cfg.adc_base.adc16 = adc.adc_w.adc_base.adc16;
    hpm_adc_init(&cfg);
    cfg.adc_base.adc16 = adc.adc_v.adc_base.adc16;
    hpm_adc_init(&cfg);

    ch_cfg.module = ADCX_MODULE_ADC16;
    hpm_adc_init_channel_default_config(&ch_cfg);

    ch_cfg.config.adc16_ch.sample_cycle = sample_cycle;

    ch_cfg.adc_base.adc16 = adc.adc_w.adc_base.adc16;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_ADC_CH_U;
    hpm_adc_channel_init(&ch_cfg);

    ch_cfg.adc_base.adc16 = adc.adc_v.adc_base.adc16;
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_ADC_CH_V;
    hpm_adc_channel_init(&ch_cfg);
    ch_cfg.config.adc16_ch.ch = BOARD_BLDC_ADC_CH_VBUS_1;
    hpm_adc_channel_init(&ch_cfg);

    init_trigger_cfg();

    /* Set DMA start address for preemption mode */
    hpm_adc_init_pmt_dma(&adc.adc_w, core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)adc.adc_w_buff));
    hpm_adc_init_pmt_dma(&adc.adc_v, core_local_mem_to_sys_address(HPM_CORE0, (uint32_t)adc.adc_v_buff));
}

void adc_set_callback(adc_callback_t cb)
{
    __isr_callback = cb;
}

uint16_t adc_GetRaw_V(void)
{
    return adc.adc_w_buff[BOARD_BLDC_ADC_CH_W_TRG * 4 + 0].value;
}

uint16_t adc_GetRaw_W(void)
{
    return adc.adc_v_buff[BOARD_BLDC_ADC_CH_V_TRG * 4 + 0].value;
}

uint16_t adc_GetRaw_VBUS(void)
{
    return adc.adc_v_buff[BOARD_BLDC_ADC_CH_V_TRG * 4 + 1].value;
}

/**
 * @brief 中断服务函数
 */
static SDK_DECLARE_EXT_ISR_M(IRQn_ADC0, __isr_adc0_fun);
static void __isr_adc0_fun(void)
{
    uint32_t status;
    adc_type adc = {adc_module_adc16, {HPM_ADC0}};
    status = hpm_adc_get_status_flags(&adc);

    if ((status & BOARD_BLDC_ADC_TRIG_FLAG) != 0)
    {
        hpm_adc_clear_status_flags(&adc, BOARD_BLDC_ADC_TRIG_FLAG);
        if (__isr_callback)
            __isr_callback(HPM_ADC0, BOARD_BLDC_ADC_TRIG_FLAG);
    }
}

static SDK_DECLARE_EXT_ISR_M(IRQn_ADC1, __isr_adc1_fun);
static void __isr_adc1_fun(void)
{
    uint32_t status;
    adc_type adc = {adc_module_adc16, {HPM_ADC1}};

    status = hpm_adc_get_status_flags(&adc);

    if ((status & BOARD_BLDC_ADC_TRIG_FLAG) != 0)
    {
        hpm_adc_clear_status_flags(&adc, BOARD_BLDC_ADC_TRIG_FLAG);
        if (__isr_callback)
            __isr_callback(HPM_ADC1, BOARD_BLDC_ADC_TRIG_FLAG);
    }
}
