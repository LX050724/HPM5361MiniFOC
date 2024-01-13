#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_MT6701 1

#define ENCODER_TYPE ENCODER_MT6701

#define USE_AUTO_SIMPLETIME 0
#define PWM_FREQUENCY (50000) /*PWM 频率  单位HZ*/
#define SPEED_PID_FREQUENCY (5000)
#define PWM_RELOAD (motor_clock_hz / PWM_FREQUENCY) /*20K hz  = 200 000 000/PWM_RELOAD */
#define ADC_IGNORE_BIT 0

#ifdef __cplusplus
}
#endif