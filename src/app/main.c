#include "SEGGER_RTT.h"
#include "app/current_calibration/current_calibration.h"
#include "app/electrical_angle_calibration/electrical_angle_calibration.h"
#include "board.h"
#include "foc/fast_sin.h"
#include "foc/foc_core.h"
#include "foc/foc_pid.h"
#include "foc/foc_pll.h"
#include "hardware/adc_init.h"
#include "hardware/can_init.h"
#include "hardware/current/current.h"
#include "hardware/encoder/encoder.h"
#include "hardware/encoder/mt6701_ssi_init.h"
#include "hardware/pwm_init.h"
#include "hardware/trgm.h"
#include "hpm_clock_drv.h"
#include "hpm_mcan_drv.h"
#include "hpm_sei_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "hpm_usb_drv.h"
#include "pinmux.h"
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

typedef struct
{
    float data[15];
    uint8_t tail[4];
} just_float_data;

#define BUF_NUM (2048 * 2 / sizeof(just_float_data))

volatile int write_ptr;
DMA_ATTR just_float_data vofa_data[BUF_NUM];

float VBUS; //!<@brief 母线电压

MotorClass_t motor0;

#if 1
static void mt6701_isr_callback(uint32_t isr_flag)
{
    uint8_t mt6701_status = encoder_get_status();

    if (mt6701_status & 0xf)
    {
        // disable_all_pwm_output();
        board_led_write(BOARD_LED_GPIO_RED_PIN, BOARD_LED_ON_LEVEL);
        SEGGER_RTT_printf(0, "MT6701 Error %x\n", mt6701_status);
    }

    // 电压保护，复位芯片恢复
    VBUS = adc_voltage(adc_GetRaw_VBUS()) * VOLTAGE_AMP;
    if (VBUS < 9 || VBUS > 30)
    {
        pwm_disable_all_output();
        board_led_write(BOARD_LED_GPIO_RED_PIN, BOARD_LED_ON_LEVEL);
        SEGGER_RTT_printf(0, "VBUS Error: %dmV\n", (int)(VBUS * 1000));
    }

    Motor_RunFoc(&motor0);

    /* 使能DTR才发送数据，方便vofa静止查看波形 */
    if (dtr_enable)
    {
        vofa_data[write_ptr].data[0] = motor0.qd_current.iq;
        vofa_data[write_ptr].data[1] = motor0.qd_current.id;
        vofa_data[write_ptr].data[2] = motor0.speed_pll.theta;
        vofa_data[write_ptr].data[3] =
            motor0.speed_pll.speed / motor0.encoder.pole_pairs / (2 * F_PI) * 60 * SPEED_PID_FREQUENCY;
        // vofa_data[write_ptr].data[5] = foc_para.currentdpipar.outval;

        vofa_data[write_ptr].data[6] = motor0.uvw_current.iu;
        vofa_data[write_ptr].data[7] = motor0.uvw_current.iv;
        vofa_data[write_ptr].data[8] = motor0.uvw_current.iw;

        vofa_data[write_ptr].data[9] = encoder_get_eleAngle(&motor0.encoder, motor0.raw_angle);

        vofa_data[write_ptr].data[10] = VBUS;
        vofa_data[write_ptr].data[11] = motor0.speed_pll._epsilon;

        // vofa_data[write_ptr].data[12] = speed_pid.cur;
        // vofa_data[write_ptr].data[14] = pll_speed * PWM_FREQUENCY * 60 / 2 / F_PI;

        // vofa_data[write_ptr].data[12] = pwm.pwm_u;
        // vofa_data[write_ptr].data[13] = pwm.pwm_v;
        // vofa_data[write_ptr].data[14] = pwm.pwm_w;

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

    // gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOB, 5, 0);
}
#endif

void motor0_get_uvw_current(MotorClass_t *motor, foc_uvw_current_t *uvw_current)
{
    current_get_cal(&motor->current_cal, adc_GetRaw_V(), adc_GetRaw_W(), uvw_current);
}

uint16_t motor0_get_raw_angle(MotorClass_t *motor)
{
    return encoder_get_rawAngle();
}

void motor0_set_pwm(MotorClass_t *motor, const foc_pwm_t *pwm)
{
    pwm_setvalue(pwm);
}

void motor0_enable_pwm(MotorClass_t *motor, bool en)
{
    en ? pwm_enable_all_output() : pwm_disable_all_output();
}

int main(void)
{
    board_init();
    board_init_led_pins();
    init_butn_pins();
    clock_cpu_delay_ms(50);

    board_init_usb_pins();
    usb_phy_using_internal_vbus(BOARD_USB);
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 2);
    cdc_acm_init();
    board_init_canfd(CAN_BAUDRATE_1M, mcan_mode_loopback_external);

    if (AHB_CLOCK != clock_get_frequency(clock_mot0))
    {
        SEGGER_RTT_printf(0, "clock error AHB_CLOCK(%d) != clock_mot0(%d)!", AHB_CLOCK,
                          clock_get_frequency(clock_mot0));
        board_led_write(BOARD_LED_GPIO_RED_PIN, BOARD_LED_ON_LEVEL);
        while (1)
            ;
    }

    for (int i = 0; i < BUF_NUM; i++)
    {
        vofa_data[i].tail[0] = 0x00;
        vofa_data[i].tail[1] = 0x00;
        vofa_data[i].tail[2] = 0x80;
        vofa_data[i].tail[3] = 0x7f;
    }

    /* PLL测速参数 */
    motor0.speed_pll.pi.kp = 0.2;
    motor0.speed_pll.pi.ki = 0.1;
    motor0.speed_pll.pi.integral_limit = 5;
    motor0.speed_pll.pi.output_limit = 2000;

    motor0.get_uvw_current_cb = motor0_get_uvw_current;
    motor0.get_raw_angle_cb = motor0_get_raw_angle;
    motor0.set_pwm_cb = motor0_set_pwm;
    motor0.enable_pwm = motor0_enable_pwm;

    Motor_Init(&motor0);

    /* PWM初始化 */
    pwm_init(20, 50);

    /* ADC初始化 */
    adc_init(30);
    /* 连接PWMCH8、ADCX_PTRGI0A */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM1_CH8REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, trgm_output_same_as_input, false);
    /* 连接PWMCH9、ADCX_PTRGI0B */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM1_CH9REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B, trgm_output_same_as_input, false);

    /* adc中值校准程序 */
    current_calibration(&motor0);

    /* 编码器初始化 */
#if ENCODER_TYPE == ENCODER_MT6701
    board_init_sei_pins(BOARD_SEI, BOARD_SEI_CTRL);
    sei_trigger_input_config_t mt6701_trigger_config = {.trig_in0_enable = true, .trig_in0_select = 0};
    // encoder_set_callback(mt6701_isr_callback);
    mt6701_ssi_init(13000000, &mt6701_trigger_config);
    intc_m_enable_irq_with_priority(BOARD_SEI_IRQn, 1);
    /* 连接PWMCH8、SEI_TRIG_IN0 */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM1_CH9REF, HPM_TRGM0_OUTPUT_SRC_SEI_TRIG_IN0, trgm_output_same_as_input, false);
#endif

    /* 电角度校准 */
    // encoder_set_param(1, 7, 26211);
    if (electrical_angle_calibration(&motor0) == 0)
    {
        Motor_SetMode(&motor0, VOLTAGE_OPEN_LOOP_MODE);
        encoder_set_callback(mt6701_isr_callback);
    }

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
    // {"speed_filter", NULL, &speed_filter},
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