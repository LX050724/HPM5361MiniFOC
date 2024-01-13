#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_sei_drv.h"
#include "hpm_soc.h"
#include "hpm_synt_drv.h"
#include <stdint.h>

static void (*__isr_callback)(uint32_t);

/**
 * @brief
 *
 * @param baud
 * @param trigger_config
 * @param callback
 * @return int
 */
int mt6701_ssi_init(uint32_t baud, sei_trigger_input_config_t *trigger_config, void (*isr_callback)(uint32_t))
{
    __isr_callback = isr_callback;

    /* [1] 传输配置 */
    sei_tranceiver_config_t tranceiver_config = {0};
    tranceiver_config.mode = sei_synchronous_master_mode; // 同步主机模式
    tranceiver_config.tri_sample = true;                  // 三重采样
    tranceiver_config.src_clk_freq = clock_get_frequency(clock_mot0);
    tranceiver_config.synchronous_master_config.data_idle_high_z = false;              // 数据空闲状态高阻设置
    tranceiver_config.synchronous_master_config.data_idle_state = sei_idle_high_state; // 数据空闲状态电平设置
    tranceiver_config.synchronous_master_config.clock_idle_high_z = false;
    tranceiver_config.synchronous_master_config.clock_idle_state = sei_idle_low_state;
    tranceiver_config.synchronous_master_config.baudrate = baud; // 波特率
    sei_tranceiver_config_init(BOARD_SEI, BOARD_SEI_CTRL, &tranceiver_config);
    uint16_t ck0_point = sei_get_xcvr_ck0_point(BOARD_SEI, BOARD_SEI_CTRL);
    uint16_t ck1_point = sei_get_xcvr_ck1_point(BOARD_SEI, BOARD_SEI_CTRL);
    // 设置数据接收点时刻(设置为时钟下降沿)
    sei_set_xcvr_rx_point(BOARD_SEI, BOARD_SEI_CTRL, ck0_point);
    // 设置数据发送点时刻(设置为时钟上升沿)
    sei_set_xcvr_tx_point(BOARD_SEI, BOARD_SEI_CTRL, ck1_point);

    /* [2] 数据寄存器配置 */
    sei_data_format_config_t data_format_config = {0};
    /* 数据寄存器1保存14bit角度信息 */
    data_format_config.mode = sei_data_mode;             // *数据模式/检查模式/CRC模式
    data_format_config.signed_flag = false;              //
    data_format_config.bit_order = sei_bit_msb_first;    // 比特顺序
    data_format_config.word_order = sei_word_nonreverse; // 字节序反转
    data_format_config.word_len = 14;                    // 字长 0-31: 1-32bit
    data_format_config.last_bit = 0;                     // 末个填充位 5b
    data_format_config.first_bit = 13;                   // 首个填充位 5b
    data_format_config.max_bit = 13;                     // 最高位 5b
    data_format_config.min_bit = 0;                      // 最低位 5b
    sei_cmd_data_format_config_init(BOARD_SEI, SEI_SELECT_DATA, SEI_DAT_2, &data_format_config);
    /* 数据寄存器2保存4bit状态 */
    data_format_config.mode = sei_data_mode;
    data_format_config.signed_flag = false;
    data_format_config.bit_order = sei_bit_msb_first;
    data_format_config.word_order = sei_word_nonreverse;
    data_format_config.word_len = 4;
    data_format_config.last_bit = 0;
    data_format_config.first_bit = 3;
    data_format_config.max_bit = 3;
    data_format_config.min_bit = 0;
    sei_cmd_data_format_config_init(BOARD_SEI, SEI_SELECT_DATA, SEI_DAT_3, &data_format_config);
    /* 数据寄存器3保存6bit CRC */
    data_format_config.mode = sei_data_mode;
    data_format_config.signed_flag = false;
    data_format_config.bit_order = sei_bit_msb_first;
    data_format_config.word_order = sei_word_nonreverse;
    data_format_config.word_len = 6;
    data_format_config.last_bit = 0;
    data_format_config.first_bit = 5;
    data_format_config.max_bit = 5;
    data_format_config.min_bit = 0;
    sei_cmd_data_format_config_init(BOARD_SEI, SEI_SELECT_DATA, SEI_DAT_4, &data_format_config);
    int index = 0;
    /* [3] sei 指令 */
    // 0：假读取拉低DE，波特率过高的时候占两个周期延长DE拉低时间到CK拉低时间(TL > 100ns)
    sei_set_instr(BOARD_SEI, index++, SEI_INSTR_OP_RECV, SEI_INSTR_M_CK_LOW, SEI_DAT_0, SEI_DAT_0, 2);
    // 1：假读取产生第一个时钟
    sei_set_instr(BOARD_SEI, index++, SEI_INSTR_OP_RECV, SEI_INSTR_M_CK_RISE_FALL, SEI_DAT_0, SEI_DAT_0, 1);
    // 2：开始读取14bit位置数据
    sei_set_instr(BOARD_SEI, index++, SEI_INSTR_OP_RECV, SEI_INSTR_M_CK_RISE_FALL, SEI_DAT_0, SEI_DAT_2, 14);
    // 3：4bit磁场状态
    sei_set_instr(BOARD_SEI, index++, SEI_INSTR_OP_RECV, SEI_INSTR_M_CK_RISE_FALL, SEI_DAT_0, SEI_DAT_3, 4);
    // 4：6bit CRC
    sei_set_instr(BOARD_SEI, index++, SEI_INSTR_OP_RECV, SEI_INSTR_M_CK_RISE_FALL, SEI_DAT_0, SEI_DAT_4, 6);
    // 5：假发送拉低时钟，拉高DE
    sei_set_instr(BOARD_SEI, index++, SEI_INSTR_OP_SEND, SEI_INSTR_M_CK_LOW, SEI_DAT_0, SEI_DAT_1, 1);

    /* [4] 状态转换配置 */
    /* latch0 */
    sei_state_transition_config_t state_transition_config = {0};
    sei_state_transition_latch_config_t state_transition_latch_config = {0};
    state_transition_config.disable_clk_check = false;                     // 检查时钟
    state_transition_config.clk_cfg = sei_state_tran_condition_fall_leave; // 0-3：时钟线1/0/上升沿/[下降沿]
    state_transition_config.disable_txd_check = true;                      // 不检查发送数据线
    state_transition_config.disable_rxd_check = true;                      // 不检查接受数据线
    state_transition_config.disable_timeout_check = true;                  // 不检查超时
    state_transition_config.disable_instr_ptr_check = true;                // 不检查指针
    sei_state_transition_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_0, SEI_CTRL_LATCH_TRAN_0_1,
                                     &state_transition_config);
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = true; // 全不检查则为空状态跳过
    sei_state_transition_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_0, SEI_CTRL_LATCH_TRAN_1_2,
                                     &state_transition_config);
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = true; // 全不检查则为空状态跳过
    sei_state_transition_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_0, SEI_CTRL_LATCH_TRAN_2_3,
                                     &state_transition_config);
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = false; // 检查指针
    state_transition_config.instr_ptr_cfg = 3;               // 0-3：匹配/不匹配/开始/[结束]
    state_transition_config.instr_ptr_value = 4;             // CRC传输完成即视为数据接收完成
    sei_state_transition_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_0, SEI_CTRL_LATCH_TRAN_3_0,
                                     &state_transition_config);

    state_transition_latch_config.enable = true;                           // 启用状态机
    state_transition_latch_config.output_select = SEI_CTRL_LATCH_TRAN_0_1; // 状态0-1时输出(时钟开始变化)
    state_transition_latch_config.delay = 0;                               // 延时0
    sei_state_transition_latch_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_0, &state_transition_latch_config);

    /* latch1 */
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = false; // 检查指针
    state_transition_config.instr_ptr_cfg = 3;               // 0-3：匹配/不匹配/开始/[结束]
    state_transition_config.instr_ptr_value = 4;             // CRC传输完成即视为数据接收完成
    sei_state_transition_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_1, SEI_CTRL_LATCH_TRAN_0_1,
                                     &state_transition_config);
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = true; // 全不检查则为空状态跳过
    sei_state_transition_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_1, SEI_CTRL_LATCH_TRAN_1_2,
                                     &state_transition_config);
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = true; // 全不检查则为空状态跳过
    sei_state_transition_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_1, SEI_CTRL_LATCH_TRAN_2_3,
                                     &state_transition_config);
    state_transition_config.disable_clk_check = true;
    state_transition_config.disable_txd_check = true;
    state_transition_config.disable_rxd_check = true;
    state_transition_config.disable_timeout_check = true;
    state_transition_config.disable_instr_ptr_check = true; // 全不检查则为空状态跳过
    sei_state_transition_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_1, SEI_CTRL_LATCH_TRAN_3_0,
                                     &state_transition_config);

    state_transition_latch_config.enable = true;                           // 启用状态机
    state_transition_latch_config.output_select = SEI_CTRL_LATCH_TRAN_0_1; // 状态0-1时输出(传输完成)
    state_transition_latch_config.delay = 0;                               // 延时0
    sei_state_transition_latch_config_init(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_1, &state_transition_latch_config);

    /* [5] 时间戳采样配置 */
    sei_sample_config_t sample_config = {0};
    sample_config.latch_select = SEI_LATCH_0; // 状态机0
    sei_sample_config_init(BOARD_SEI, BOARD_SEI_CTRL, &sample_config);

    /* [6] 传输完成更新配置 */
    sei_update_config_t update_config = {0};
    update_config.pos_data_use_rx = true;           // 包含位置
    update_config.pos_data_idx = SEI_DAT_1;         // 位置寄存器选择1
    update_config.rev_data_use_rx = false;          // 不包含圈数
    update_config.spd_data_use_rx = false;          // 不包含速度
    update_config.acc_data_use_rx = false;          // 不包含加速度
    update_config.update_on_err = false;            // 出错时不更新
    update_config.latch_select = SEI_LATCH_1;       // 状态机1
    update_config.data_register_select = BIT2_MASK; /* 选择数据寄存器 */
    sei_update_config_init(BOARD_SEI, BOARD_SEI_CTRL, &update_config);

    // 状态机1事件和错误事件中断
    sei_set_irq_enable(BOARD_SEI, BOARD_SEI_CTRL, sei_irq_latch1_event | sei_irq_trx_err_event, true);

    /* [7] 启用同步定时器时间戳 */
    synt_enable_timestamp(HPM_SYNT, true);

    /* [8] 引擎配置 */
    sei_engine_config_t engine_config = {0};
    engine_config.arming_mode = sei_arming_wait_trigger;
    engine_config.data_cdm_idx = 0;
    engine_config.data_base_idx = 0;
    engine_config.init_instr_idx = 0;
    engine_config.wdg_enable = false;
    sei_engine_config_init(BOARD_SEI, BOARD_SEI_CTRL, &engine_config);
    sei_set_engine_enable(BOARD_SEI, BOARD_SEI_CTRL, true);

    /* [10] 触发配置 */
    sei_trigger_input_config_init(BOARD_SEI, BOARD_SEI_CTRL, trigger_config);

    return 0;
}

/**
 * @brief 中断服务函数
 */
static SDK_DECLARE_EXT_ISR_M(BOARD_SEI_IRQn, __isr_sei_fun);
static void __isr_sei_fun(void)
{
    if (sei_get_irq_status(BOARD_SEI, BOARD_SEI_CTRL, sei_irq_latch1_event))
    {
        sei_clear_irq_flag(BOARD_SEI, BOARD_SEI_CTRL, sei_irq_latch1_event);
        __isr_callback(sei_irq_latch1_event);
    }

    if (sei_get_irq_status(BOARD_SEI, BOARD_SEI_CTRL, sei_irq_trx_err_event))
    {
        sei_clear_irq_flag(BOARD_SEI, BOARD_SEI_CTRL, sei_irq_trx_err_event);
        __isr_callback(sei_irq_trx_err_event);
    }
}