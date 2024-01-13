#include "SEGGER_RTT.h"
#include "board.h"
#include "foc/fast_sin.h"
#include "foc/foc_core.h"
#include "hardware/adc_init.h"
#include "hardware/can_init.h"
#include "hardware/encoder/mt6701_ssi_init.h"
#include "hardware/pwm_init.h"
#include "hardware/trgm.h"
#include "hpm_adc.h"
#include "hpm_adc16_regs.h"
#include "hpm_clock_drv.h"
#include "hpm_mcan_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_sei_drv.h"
#include "hpm_soc.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "hpm_usb_drv.h"
#include "project_config.h"
#include "stdbool.h"
#include "usb_config.h"
#include "usb_dc.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint8_t dtr_enable;
extern volatile uint8_t rts_enable;
extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);
extern volatile bool ep_tx_busy_flag;

// void foc_pi_contrl(BLDC_CONTRL_PID_PARA *par);

typedef struct
{
    float data[15];
    uint8_t tail[4];
} just_float_data;

#define BUF_NUM (2048 * 2 / sizeof(just_float_data))

int16_t ele_ang_offset = 0x1100 - 16384;

volatile int write_ptr;
DMA_ATTR just_float_data vofa_data[BUF_NUM];

DMA_ATTR CurrentADC_t adc;

float adc_IW; //!<@brief U相电流
float adc_IV; //!<@brief V相电流
float adc_IU; //!<@brief W相电流
float VBUS;   //!<@brief 母线电压
float CUR;    //!<@brief 母线电流

volatile int adc_calibration_status = 0;
int adc_calibration_u = 0;
int adc_calibration_v = 0;
int adc_calibration_i = 0;
uint32_t adc_calibration_num = 0;

int32_t adc_raw_v;
int32_t adc_raw_v_last;
int32_t adc_raw_w;
int32_t adc_raw_w_last;
int32_t adc_raw_i;
int32_t adc_raw_vbus;

int16_t mt6701_ang = 0;
int16_t mt6701_lst_ang = 0;
uint8_t mt6701_status = 0;

// BLDC_CONTRL_PID_PARA speed_pid;
float speed, speed_last;
float speed_filter = 1 / 0.02;
int intr_count;

float ang;
foc_alpha_beta_volt_t ab;
foc_pwm_t pwm;
foc_qd_current_t out_qd_current = {0.5};

int32_t motor_clock_hz;

void foc_pwmset(uint32_t pwm_reload, foc_pwm_t *par)
{
    uint32_t pwm_reload_half;
    uint32_t pwm_u_half, pwm_v_half, pwm_w_half;

    pwm_reload_half = pwm_reload >> 1;
    pwm_u_half = par->pwm_u >> 1;
    pwm_v_half = par->pwm_v >> 1;
    pwm_w_half = par->pwm_w >> 1;
    pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_0, PWM_CMP_CMP_SET((pwm_reload_half + pwm_u_half)));
    pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_1, PWM_CMP_CMP_SET((pwm_reload_half - pwm_u_half)));
    pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_2, PWM_CMP_CMP_SET((pwm_reload_half + pwm_v_half)));
    pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_3, PWM_CMP_CMP_SET((pwm_reload_half - pwm_v_half)));
    pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_4, PWM_CMP_CMP_SET((pwm_reload_half + pwm_w_half)));
    pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_5, PWM_CMP_CMP_SET((pwm_reload_half - pwm_w_half)));
#if USE_AUTO_SIMPLETIME
    if (par->pwm_u < pwm_reload_half)
        trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, HPM_TRGM0_INPUT_SRC_PWM1_CH8REF);
    else
        trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, HPM_TRGM0_INPUT_SRC_PWM1_CH9REF);

    if (par->pwm_v < pwm_reload_half)
        trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B, HPM_TRGM0_INPUT_SRC_PWM1_CH8REF);
    else
        trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B, HPM_TRGM0_INPUT_SRC_PWM1_CH9REF);
#endif
}

static void adc_isr_callback(ADC16_Type *padc, uint32_t status)
{
    if (adc_calibration_status == 1)
    {
        adc_calibration_u += adc.adc_w_buff[BOARD_BLDC_ADC_CH_U_TRG * 4 + 0].value;
        adc_calibration_v += adc.adc_v_buff[BOARD_BLDC_ADC_CH_V_TRG * 4 + 0].value;
        adc_calibration_num++;
        if (adc_calibration_num == 1024)
        {
            adc_calibration_u /= 1024;
            adc_calibration_v /= 1024;
            adc_calibration_status = 2;
        }
        return;
    }
}

int count = 0;

static void mt6701_isr_callback(uint32_t isr_flag)
{
    mt6701_ang = sei_get_data_value(BOARD_SEI, SEI_DAT_2);
    mt6701_status = sei_get_data_value(BOARD_SEI, SEI_DAT_3);

    if (mt6701_status & 0xf)
    {
        // disable_all_pwm_output();
        board_led_write(BOARD_LED_GPIO_RED_PIN, BOARD_LED_ON_LEVEL);
        SEGGER_RTT_printf(0, "MT6701 Error %x\n", mt6701_status);
    }

    int16_t mt6701_diff = mt6701_ang - mt6701_lst_ang;
    if (mt6701_diff > 8192)
        mt6701_diff -= 16384;
    if (mt6701_diff < -8192)
        mt6701_diff += 16384;
    mt6701_lst_ang = mt6701_ang;
    speed += ((mt6701_diff / 16384.0f * PWM_FREQUENCY * 60) - speed_last) / speed_filter;
    if (speed > 1000)
        speed = 1000;
    if (speed < -1000)
        speed = -1000;
    speed_last = speed;

    float electric_angle = F_PI - ((mt6701_ang - ele_ang_offset) * 7 / 16384.0f * 2 * F_PI);

    adc_raw_w = adc.adc_w_buff[BOARD_BLDC_ADC_CH_U_TRG * 4 + 0].value;
    adc_raw_v = adc.adc_v_buff[BOARD_BLDC_ADC_CH_V_TRG * 4 + 0].value;
    adc_raw_vbus = adc.adc_v_buff[BOARD_BLDC_ADC_CH_V_TRG * 4 + 1].value;

    adc_raw_w_last = adc_raw_w;
    adc_raw_v_last = adc_raw_v;
    adc_IW = ((((adc_raw_w + adc_raw_w_last) / 2 - adc_calibration_u) >> ADC_IGNORE_BIT) * 3.3f /
              (0xffff >> ADC_IGNORE_BIT)) /
             -0.1f;
    adc_IV = ((((adc_raw_v + adc_raw_v_last) / 2 - adc_calibration_v) >> ADC_IGNORE_BIT) * 3.3f /
              (0xffff >> ADC_IGNORE_BIT)) /
             -0.1f;
    adc_IU = 0 - adc_IW - adc_IV;

    VBUS = ((adc_raw_vbus >> ADC_IGNORE_BIT) * 3.3f / (0xffff >> ADC_IGNORE_BIT)) * 11.0f;

    foc_uvw_current_t uvw_current = {adc_IU, adc_IV, adc_IW};

    foc_alpha_beta_current_t alpha_beta_current;
    foc_sin_cos_t sin_cos;
    foc_qd_current_t qd_current;

    foc_sin_cos(electric_angle, &sin_cos);
    foc_clarke(&uvw_current, &alpha_beta_current);
    foc_park(&alpha_beta_current, &sin_cos, &qd_current);

    // TODO pid ...

    foc_inv_park(&out_qd_current, &sin_cos, &ab);
    foc_svpwm(&ab, &pwm, PWM_RELOAD * 0.9);
    foc_pwmset(PWM_RELOAD, &pwm);

    // 电压保护，复位芯片恢复
    if (VBUS < 9 || VBUS > 30)
    {
        disable_all_pwm_output();
        board_led_write(BOARD_LED_GPIO_RED_PIN, BOARD_LED_ON_LEVEL);
        SEGGER_RTT_printf(0, "VBUS Error\n");
    }

    /* 使能DTR才发送数据，方便vofa静止查看波形 */
    if (dtr_enable)
    {
        vofa_data[write_ptr].data[0] = qd_current.iq;
        vofa_data[write_ptr].data[1] = qd_current.id;
        // vofa_data[write_ptr].data[2] = iq;
        // vofa_data[write_ptr].data[3] = id;
        // vofa_data[write_ptr].data[5] = foc_para.currentdpipar.outval;

        vofa_data[write_ptr].data[6] = adc_IW;
        vofa_data[write_ptr].data[7] = adc_IV;
        vofa_data[write_ptr].data[8] = adc_IU;

        vofa_data[write_ptr].data[9] = electric_angle;

        vofa_data[write_ptr].data[10] = VBUS;
        // vofa_data[write_ptr].data[11] = CUR;

        // vofa_data[write_ptr].data[12] = speed_pid.cur;
        // vofa_data[write_ptr].data[13] = _x;
        //  vofa_data[write_ptr].data[14] = pll_speed * PWM_FREQUENCY * 60 / 2 / F_PI;

        // vofa_data[write_ptr].data[12] = foc_para.pwmpar.pwmout.pwm_u;
        // vofa_data[write_ptr].data[14] = foc_para.pwmpar.pwmout.pwm_w;

        // vofa_data[write_ptr].data[14] = (float)hpm_csr_get_core_cycle() / hpm_core_clock;

        if (write_ptr == 0)
        {
            if (!ep_tx_busy_flag)
            {
                ep_tx_busy_flag = true;
                usbd_ep_start_write(0x81, &vofa_data[BUF_NUM / 2], sizeof(just_float_data) * BUF_NUM / 2);
            }
        }

        if (write_ptr == BUF_NUM / 2)
        {
            if (!ep_tx_busy_flag)
            {
                ep_tx_busy_flag = true;
                usbd_ep_start_write(0x81, &vofa_data[0], sizeof(just_float_data) * BUF_NUM / 2);
            }
        }
        write_ptr = (write_ptr + 1) % BUF_NUM;
    }

    if (++intr_count >= (PWM_FREQUENCY / SPEED_PID_FREQUENCY))
    {
        // speed_pid.cur = speed;
        // speed_pid.func_pid(&speed_pid);
        // foc_para.currentqpipar.target = speed_pid.outval;
        // intr_count = 0;
    }

    // gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOB, 5, 0);
}

// void foc_pi_contrl(BLDC_CONTRL_PID_PARA *par)
// {
//     float result = 0;

//     float curerr = par->target - par->cur;
//     float int_max = (par->i_max / par->i_ki);
//     par->mem += curerr;

//     if (par->mem > int_max)
//         par->mem = int_max;

//     if (par->mem < -int_max)
//         par->mem = -int_max;

//     result = par->i_kp * curerr + par->i_ki * par->mem;

//     if (result < -par->i_max)
//     {
//         result = -par->i_max;
//     }
//     else if (result > par->i_max)
//     {
//         result = par->i_max;
//     }

//     par->outval = result;
// }

int main(void)
{
    board_init();
    motor_clock_hz = clock_get_frequency(clock_mot0);
    clock_cpu_delay_ms(50);
    usb_phy_using_internal_vbus(BOARD_USB);

    board_init_usb_pins();
    board_init_can_pins(BOARD_APP_CAN_BASE);
    board_init_canfd(CAN_BAUDRATE_1M, mcan_mode_loopback_external);
    board_init_led_pins();
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 2);
    cdc_acm_init();

    for (int i = 0; i < BUF_NUM; i++)
    {
        vofa_data[i].tail[0] = 0x00;
        vofa_data[i].tail[1] = 0x00;
        vofa_data[i].tail[2] = 0x80;
        vofa_data[i].tail[3] = 0x7f;
    }

    /* PWM ADC初始化 */
    pwm_init(PWM_FREQUENCY, 20, 50);

    /* ADC初始化&中值校准 */
    disable_all_pwm_output();
    adc.adc_w.adc_base.adc16 = BOARD_BLDC_ADC_U_BASE;
    adc.adc_w.module = adc_module_adc16;
    adc.adc_v.adc_base.adc16 = BOARD_BLDC_ADC_V_BASE;
    adc.adc_v.module = adc_module_adc16;

    HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PB10].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    board_init_adc16_clock(HPM_ADC0, true);
    board_init_adc16_clock(HPM_ADC1, true);
    adc_init(&adc, 30, adc_isr_callback);
    /* 连接PWMCH8、ADCX_PTRGI0A */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM1_CH8REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, trgm_output_same_as_input, false);
    /* 连接PWMCH9、ADCX_PTRGI0B */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM1_CH9REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B, trgm_output_same_as_input, false);

    intc_m_enable_irq_with_priority(IRQn_ADC0, 1);
    intc_m_enable_irq_with_priority(IRQn_ADC1, 1);
    hpm_adc_enable_interrupts(&adc.adc_v, adc16_event_trig_complete);
    adc_calibration_status = 1;
    while (adc_calibration_status != 2)
    {
        clock_cpu_delay_ms(1);
    }
    hpm_adc_disable_interrupts(&adc.adc_v, adc16_event_trig_complete);
    enable_all_pwm_output();

    /* 编码器初始化 */
    board_init_sei_pins(BOARD_SEI, BOARD_SEI_CTRL);
    sei_trigger_input_config_t mt6701_trigger_config = {.trig_in0_enable = true, .trig_in0_select = 0};
    mt6701_ssi_init(13000000, &mt6701_trigger_config, mt6701_isr_callback);
    intc_m_enable_irq_with_priority(BOARD_SEI_IRQn, 1);
    /* 连接PWMCH8、SEI_TRIG_IN0 */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM1_CH9REF, HPM_TRGM0_OUTPUT_SRC_SEI_TRIG_IN0, trgm_output_same_as_input, false);

    int count = 0;
    while (1)
    {
        clock_cpu_delay_ms(1);
        if (count++ > 300)
        {
            count = 0;
            board_led_toggle(BOARD_LED_GPIO_GREEN_PIN);
        }
        mcan_tx_frame_t can_frame = {};
        uint32_t fifo_index = 0;
        can_frame.dlc = 8;
        can_frame.std_id = 0x200;
        can_frame.event_fifo_control = 1;
        can_frame.data_32[0] = 0x12345678;
        can_frame.data_32[1] = 0xaa55aa55;
        can_frame.message_marker_l = 0;
        // mcan_transmit_via_txfifo_nonblocking(BOARD_APP_CAN_BASE, &can_frame, &fifo_index);

        // ang += 0.01;
        foc_qd_current_t qd_current = {
            .iq = 0.1,
        };

        // foc_sin_cos_t sin_cos;
        // foc_sin_cos(ang, &sin_cos);
        // foc_inv_park(&qd_current, &sin_cos, &ab);
        // // ab.v_alpha = 0.2;
        // foc_svpwm(&ab, &pwm, PWM_RELOAD * 0.9);
        // foc_pwmset(BLDC_MOTOR0_INDEX, PWM_RELOAD, &pwm);
    }
    return 0;
}

typedef struct
{
    const char *name;
    void (*fun)(float);
    float *tar_val;
} CmdCallback_t;

CmdCallback_t cmd_list[] = {
    // {"exp_iq", NULL, &foc_para.currentqpipar.target},
    // {"id_p", NULL, &foc_para.currentdpipar.i_kp},
    // {"id_i", NULL, &foc_para.currentdpipar.i_ki},
    // {"iq_p", NULL, &foc_para.currentqpipar.i_kp},
    // {"iq_i", NULL, &foc_para.currentqpipar.i_ki},
    // {"speed_p", NULL, &speed_pid.i_kp},
    // {"speed_i", NULL, &speed_pid.i_ki},
    // {"exp_speed", NULL, &speed_pid.target},
    // {"exp_id", NULL, &foc_para.currentdpipar.target},
    {"speed_filter", NULL, &speed_filter},
};

void usbd_read_callback(char *data, uint32_t len)
{
    char name[64] = {};
    float value = 0;
    char *start = data;
    for (int i = 0; i < len - 1; i++)
    {
        if (data[i] == ':')
        {
            strncpy(name, start, data + i - start);
            value = strtof(&data[i + 1], &start);
            if (start[0] == '\n')
                start++;
            for (int index = 0; index < sizeof(cmd_list) / sizeof(CmdCallback_t); index++)
            {
                if (strcmp(cmd_list[index].name, name) == 0)
                {
                    SEGGER_RTT_printf(0, "set %s %f\n", cmd_list[index].name, value);
                    if (cmd_list[index].fun == NULL)
                    {
                        *(cmd_list[index].tar_val) = value;
                    }
                    else
                    {
                        cmd_list[index].fun(value);
                    }
                    break;
                }
            }
        }
    }
}