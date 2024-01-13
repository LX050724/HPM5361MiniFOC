
//#include "board.h"
//#include "hpm_qeiv2_drv.h"

//static volatile int s_pos_cmp_matched;
//static volatile int s_pulse0_matched;
//static volatile int s_pulse1_matched;
//static volatile int s_cycle0_matched;
//static volatile int s_cycle1_matched;

//int qeiv2_uvw_init()
//{
//    qeiv2_uvw_config_t uvw_config = {};
//    qeiv2_phcnt_cmp_match_config_t phcnt_cmp_config = {};

//    // 初始化qeiv2接口
//    init_qeiv2_uvw_pins(BOARD_BLDC_QEIV2_BASE);

//    // 复位计数器，并保持复位状态
//    qeiv2_reset_counter(BOARD_BLDC_QEIV2_BASE);

//    // 编码器UVW模式
//    qeiv2_set_work_mode(BOARD_BLDC_QEIV2_BASE, qeiv2_work_mode_uvw);

//    // 配置信号启用和边缘，UVW霍尔模式全部启用
//    qeiv2_config_abz_uvw_signal_edge(BOARD_BLDC_QEIV2_BASE, true, true, true, true, true);

//    // 获取uvw位置参数的默认配置
//    qeiv2_get_uvw_position_defconfig(&uvw_config);
//    // uvw_config.pos_opt = qeiv2_uvw_pos_opt_next;

//    // 配置uvw配置参数
//    qeiv2_config_uvw_position(BOARD_BLDC_QEIV2_BASE, &uvw_config);

//    //
//    qeiv2_select_spd_tmr_register_content(BOARD_BLDC_QEIV2_BASE, qeiv2_spd_tmr_as_spd_tm);

//    // 计数器溢出计圈
//    qeiv2_config_z_phase_counter_mode(BOARD_BLDC_QEIV2_BASE, qeiv2_z_count_inc_on_z_input_assert);

//    // 脉冲计数器最大值
//    qeiv2_config_phmax_phparam(BOARD_BLDC_QEIV2_BASE, 42);

//    phcnt_cmp_config.ignore_zcmp = true;
//    phcnt_cmp_config.ignore_rotate_dir = true;
//    phcnt_cmp_config.phcnt_cmp_value = 6;
//    qeiv2_config_phcnt_cmp_match_condition(BOARD_BLDC_QEIV2_BASE, &phcnt_cmp_config);
//    // qeiv2_enable_load_read_trigger_event(BOARD_BLDC_QEIV2_BASE, QEIV2_EVENT_POSITION_COMPARE_FLAG_MASK);

//    // 每6个脉冲更新一次，开启中断
//    qeiv2_set_pulse0_num(BOARD_BLDC_QEIV2_BASE, 6);
//    qeiv2_enable_irq(BOARD_BLDC_QEIV2_BASE, QEIV2_EVENT_PULSE0_FLAG_MASK);

//    // 5000Hz固定时间更新，开启中断
//    uint32_t clock_freq = clock_get_frequency(BOARD_BLDC_QEI_CLOCK_SOURCE);
//    qeiv2_set_cycle0_num(BOARD_BLDC_QEIV2_BASE, clock_freq / 5000);
//    qeiv2_enable_irq(BOARD_BLDC_QEIV2_BASE, QEIV2_EVENT_CYCLE0_FLAG_MASK);

//    // 启用中断
//    intc_m_enable_irq_with_priority(BOARD_BLDC_QEIV2_IRQ, 1);

//    // 启动
//    qeiv2_release_counter(BOARD_BLDC_QEIV2_BASE);
//    return 0;
//}

///**
// * @brief qeiv2中断服务函数
// *
// */
//SDK_DECLARE_EXT_ISR_M(BOARD_BLDC_QEIV2_IRQ, isr_qei)
//void isr_qei(void)
//{
//    uint32_t status = qeiv2_get_status(BOARD_BLDC_QEIV2_BASE);

//    qeiv2_clear_status(BOARD_BLDC_QEIV2_BASE, status);

//    if ((status & QEIV2_EVENT_POSITION_COMPARE_FLAG_MASK) != 0)
//    {
//        s_pos_cmp_matched = !s_pos_cmp_matched;
//    }

//    if ((status & QEIV2_EVENT_PULSE0_FLAG_MASK) != 0)
//    {
//        s_pulse0_matched = !s_pulse0_matched;
//    }

//    if ((status & QEIV2_EVENT_PULSE1_FLAG_MASK) != 0)
//    {
//        s_pulse1_matched = !s_pulse1_matched;
//    }

//    if ((status & QEIV2_EVENT_CYCLE0_FLAG_MASK) != 0)
//    {
//        s_cycle0_matched++;
//    }

//    if ((status & QEIV2_EVENT_CYCLE1_FLAG_MASK) != 0)
//    {
//        s_cycle1_matched++;
//    }
//}