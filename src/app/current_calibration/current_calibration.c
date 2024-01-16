#include "current_calibration.h"
#include "SEGGER_RTT.h"
#include "board.h"
#include "hardware/adc_init.h"
#include "hardware/pwm_init.h"
#include "hpm_clock_drv.h"
#include "project_config.h"
#include <stdint.h>

static volatile int calibration_status;
static volatile uint32_t calibration_v;
static volatile uint32_t calibration_w;
static volatile uint32_t calibration_count;
static MotorClass_t *gpMotor;

static void __adc_cb(ADC16_Type *adc, uint32_t flag)
{
    if (adc == BOARD_BLDC_ADC_V_BASE)
    {
        calibration_v += adc_GetRaw_V();
        calibration_w += adc_GetRaw_W();
        if (++calibration_count >= ADC_CALIBRATION_TIMES)
        {
            calibration_v /= ADC_CALIBRATION_TIMES;
            calibration_w /= ADC_CALIBRATION_TIMES;
            current_set_calibration(&gpMotor->current_cal, calibration_v, calibration_w);
            calibration_status = 2;
        }
    }
}

int current_calibration(MotorClass_t *motor)
{
    pwm_disable_all_output();
    clock_cpu_delay_ms(500);

    gpMotor = motor;
    calibration_v = 0;
    calibration_w = 0;
    calibration_count = 0;
    calibration_status = 1;
    adc_set_callback(__adc_cb);
    adc_enable_irq(1);
    adc_enable_it();

    while (calibration_status != 2)
    {
    }

    adc_disable_it();
    adc_disable_irq();
    adc_set_callback(NULL);
    gpMotor = NULL;

    SEGGER_RTT_printf(0, "current calibration done\n");
    SEGGER_RTT_printf(0, "  V offset %dmV\n", (int)(adc_voltage(calibration_v) * 1000));
    SEGGER_RTT_printf(0, "  W offset %dmV\n", (int)(adc_voltage(calibration_w) * 1000));
    return 0;
}