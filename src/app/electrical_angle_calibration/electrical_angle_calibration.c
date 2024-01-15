#include "SEGGER_RTT.h"
#include "foc/fast_sin.h"
#include "foc/foc_core.h"
#include "hardware/encoder/encoder.h"
#include "hardware/pwm_init.h"
#include "hpm_clock_drv.h"
#include <math.h>
#include <stdint.h>

void force_drive(float ang, float power)
{
    foc_sin_cos_t ang_sincos = {};
    foc_qd_current_t qd_current = {};
    foc_alpha_beta_volt_t alpha_beta_volt = {};
    foc_pwm_t pwm = {};

    qd_current.id = power;
    foc_sin_cos(ang, &ang_sincos);
    foc_inv_park(&qd_current, &ang_sincos, &alpha_beta_volt);
    foc_svpwm(&alpha_beta_volt, &pwm);
    pwm_setvalue(&pwm);
}

float LeastSquareLinearFit(int y[], const int num, float *a, float *b)
{
    float sum_x2 = 0;
    float sum_y = 0;
    float sum_x = 0;
    float sum_xy = 0;
    float sum_q = 0;

    for (int i = 0; i < num; i++)
    {
        float x = F_PI * i / 6;
        sum_x2 += x * x;
        sum_y += y[i];
        sum_x += x;
        sum_xy += x * y[i];
    }

    *a = (sum_x2 * sum_y - sum_x * sum_xy) / (num * sum_x2 - sum_x * sum_x);
    *b = (num * sum_xy - sum_x * sum_y) / (num * sum_x2 - sum_x * sum_x);

    for (int i = 0; i < num; i++)
    {
        float ya = y[i] - (*a) - (*b) * (F_PI * i / 6);
        sum_q += ya * ya;
    }

    return sqrtf(sum_q / num);
}

int electrical_angle_calibration()
{
    pwm_enable_all_output();
    int ang_table[12];

    for (int i = 0; i < 12; i++)
    {
        force_drive(F_PI * i / 6, ELECTRICAL_ANGLE_CALIBRATION_POWER);
        clock_cpu_delay_ms(ELECTRICAL_ANGLE_CALIBRATION_DELAY);
        ang_table[i] = encoder_get_rawAngle();
        SEGGER_RTT_printf(0, "1 %2d 0x%04x\n", i, ang_table[i]);
    }

    force_drive(0, ELECTRICAL_ANGLE_CALIBRATION_POWER);
    clock_cpu_delay_ms(ELECTRICAL_ANGLE_CALIBRATION_DELAY);

    for (int i = 11; i >= 0; i--)
    {
        force_drive(F_PI * i / 6, ELECTRICAL_ANGLE_CALIBRATION_POWER);
        clock_cpu_delay_ms(ELECTRICAL_ANGLE_CALIBRATION_DELAY);
        ang_table[i] = (ang_table[i] + encoder_get_rawAngle()) / 2;
        SEGGER_RTT_printf(0, "2 %2d 0x%04x 0x%04x\n", i, encoder_get_rawAngle(), ang_table[i]);
    }

    /* 处理溢出 */
    for (int i = 0; i < 11; i++)
    {
        int diff = ang_table[i] - ang_table[i + 1];
        if (diff > INT16_MAX)
            ang_table[i + 1] += 65536;
        else if (diff < INT16_MIN)
            ang_table[i + 1] -= 65536;
    }

    pwm_disable_all_output();

    /* 线性回归 */
    float a, b;
    float q = LeastSquareLinearFit(ang_table, 12, &a, &b);
    SEGGER_RTT_printf(0, "LeastSquareLinearFit: y = %d%+dx, Q=%d\n", (int)a, (int)b, (int)q);

    if (q > 40)
    {
        SEGGER_RTT_printf(0, "calibration error: the error is too large, Q=%d\n", (int)q);
        return -1;
    }

    int16_t pole_pairs = 1 / (fabsf(b) * 2 * F_PI / 65536.0f) + 0.5f;
    uint16_t offset = a + 0.5f;
    if (b < 0)
    {
        offset += 65536.0f / 2 / pole_pairs;
        pole_pairs = -pole_pairs;
    }
    SEGGER_RTT_printf(0, "calibration success: direction = %d, offset = %d, pole_pairs = %d\n", b > 0, offset,
                      pole_pairs);
    encoder_set_param(pole_pairs, offset);
    return 0;
}