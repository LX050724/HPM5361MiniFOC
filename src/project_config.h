#pragma once

#include "hpm_common.h"
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 编码器接口类型 */
#define ENCODER_INTERFACE_SSI 1
#define ENCODER_INTERFACE_IIC 2
#define ENCODER_INTERFACE_SPI 3
#define ENCODER_INTERFACE_QEI 4
#define ENCODER_INTERFACE_LINEAR_HALL 5
#define ENCODER_INTERFACE_ANALOG 6

/* 编码器类型 */
#define ENCODER_MT6701 (ENCODER_INTERFACE_SSI << 8 | 1)
#define ENCODER_UVW (ENCODER_INTERFACE_QEI << 8 | 2)
#define ENCODER_ABZ (ENCODER_INTERFACE_QEI << 8 | 3)
#define ENCODER_LINEAR_HALL (ENCODER_INTERFACE_LINEAR_HALL << 8 | 4)

#define ENCODER_TYPE ENCODER_MT6701 // 编码器类型

/* 时间参数 */
#define PWM_FREQUENCY 50000                     // PWM频率(Hz)
#define SPEED_PID_FREQUENCY 5000                // 速度、位置环频率(Hz)
#define PWM_MAX_DUTY 0.96f                      // PWM最大占空比不建议超过96%
#define ELECTRICAL_ANGLE_CALIBRATION_POWER 0.4f // 电角度校准油门
#define ELECTRICAL_ANGLE_CALIBRATION_DELAY 500  // 电角度校准延迟(ms)

/* ADC参数 */
#define SAMPLING_RESISTOR 0.005f                      // 采样电阻 Ω
#define CURRENT_AMP 20.0f                             // 电流运放放大倍数
#define CURRENT_COE (SAMPLING_RESISTOR * CURRENT_AMP) // 电流系数 V/A
#define VOLTAGE_AMP 11.0f                             // 母线电压放大倍数
#define ADC_IGNORE_BIT 0                              // ADC低位舍弃
#define ADC_ENABLE_FILTER 0                           // 启用2位滑动平均滤波
#define ADC_CALIBRATION_TIMES 1024                    // ADC校准采样次数
// #define USE_AUTO_SIMPLETIME 0                         // 自动切换采样时刻开关

/* 不可修改 */
#define AHB_CLOCK 240000000                    // AHB时钟频率(Hz)
#define PWM_RELOAD (AHB_CLOCK / PWM_FREQUENCY) // PWM重载值

#if ADC_CALIBRATION_TIMES > 65536
#error "ADC_CALIBRATION_TIMES > 65536"
#endif

#define GET_ENCODER_INTERFACE(x) (((x) >> 8) & 0xff)
#define DMA_ATTR ATTR_PLACE_AT_NONCACHEABLE ATTR_ALIGN(ADC_SOC_DMA_ADDR_ALIGNMENT)

#ifdef __cplusplus
}
#endif