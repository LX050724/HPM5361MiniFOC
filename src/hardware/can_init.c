#include "SEGGER_RTT.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_mcan_drv.h"
#include <stdint.h>
#include <string.h>

void board_init_canfd(uint32_t baudrate, mcan_node_mode_t mode)
{
    board_init_can_pins(BOARD_APP_CAN_BASE);
    clock_add_to_group(clock_can0, 0);
    clock_set_source_divider(clock_can0, clk_src_pll0_clk0, 10);
    uint32_t freq = clock_get_frequency(clock_can0);

    mcan_config_t can_config;
    mcan_filter_elem_t can_filters[16];
    mcan_get_default_config(BOARD_APP_CAN_BASE, &can_config);
    can_filters[0].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;
    can_filters[0].filter_config = MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH;
    can_filters[0].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;
    can_filters[0].sync_message = 1;
    can_filters[0].filter_id = 0x0;
    can_filters[0].filter_mask = 0x0;

    can_config.baudrate = baudrate;
    can_config.mode = mode;
    can_config.all_filters_config.std_id_filter_list.filter_elem_list = &can_filters[0];
    can_config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 1;
    can_config.all_filters_config.global_filter_config.accept_non_matching_std_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    can_config.all_filters_config.global_filter_config.accept_non_matching_ext_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    can_config.all_filters_config.global_filter_config.reject_remote_std_frame = true;
    can_config.all_filters_config.global_filter_config.reject_remote_ext_frame = true;
    can_config.txbuf_trans_interrupt_mask = ~0UL;
    can_config.interrupt_mask = MCAN_EVENT_RECEIVE;
    hpm_stat_t status = mcan_init(BOARD_APP_CAN_BASE, &can_config, freq);
    SEGGER_RTT_printf(0, "mcan_init status = %d\n", status);

    mcan_enable_interrupts(BOARD_APP_CAN_BASE, MCAN_EVENT_RECEIVE);
    mcan_enable_txbuf_transmission_interrupt(BOARD_APP_CAN_BASE, ~0UL);

    intc_m_enable_irq_with_priority(BOARD_APP_CAN_IRQn, 1);
}

SDK_DECLARE_EXT_ISR_M(BOARD_APP_CAN_IRQn, board_canfd_isr);
void board_canfd_isr()
{
    mcan_rx_message_t s_can_rx_buf;
    uint32_t flags = mcan_get_interrupt_flags(BOARD_APP_CAN_BASE);

    /* New message is available in RXFIFO0 */
    if ((flags & MCAN_INT_RXFIFI0_NEW_MSG) != 0)
    {
        mcan_read_rxfifo(BOARD_APP_CAN_BASE, 0, &s_can_rx_buf);
        SEGGER_RTT_printf(0, "MCAN_INT_RXFIFI0_NEW_MSG %#x\n", s_can_rx_buf.std_id);
    }

    /* New message is available in RXFIFO1 */
    if ((flags & MCAN_INT_RXFIFO1_NEW_MSG) != 0U)
    {
        mcan_read_rxfifo(BOARD_APP_CAN_BASE, 0, &s_can_rx_buf);
        SEGGER_RTT_printf(0, "MCAN_INT_RXFIFO1_NEW_MSG %#x\n", s_can_rx_buf.std_id);
    }

    /* New message is available in RXBUF */
    if ((flags & MCAN_INT_MSG_STORE_TO_RXBUF) != 0U)
    {
        SEGGER_RTT_printf(0, "MCAN_INT_MSG_STORE_TO_RXBUF %#x\n", s_can_rx_buf.std_id);
    }

    /* New TX Event occurred */
    if ((flags & MCAN_INT_TX_EVT_FIFO_NEW_ENTRY) != 0)
    {
    }

    /* Transmit completed */
    if ((flags & MCAN_EVENT_TRANSMIT) != 0U)
    {
    }
    
    /* Error happened */
    if ((flags & MCAN_EVENT_ERROR) != 0)
    {
    }
    mcan_clear_interrupt_flags(BOARD_APP_CAN_BASE, flags);
}

int board_can_transmit_std_txfifo_nonblocking(uint16_t std_id, void *data, uint8_t dlc)
{
    mcan_tx_frame_t can_frame = {};
    uint32_t fifo_index = 0;
    can_frame.dlc = dlc;
    can_frame.std_id = std_id;
    can_frame.event_fifo_control = 1;
    can_frame.message_marker_l = 0;
    memcpy(can_frame.data_8, data, dlc);

    int ret = mcan_transmit_via_txfifo_nonblocking(BOARD_APP_CAN_BASE, &can_frame, &fifo_index);
    if (ret != status_success)
        return -ret;
    return fifo_index;
}

int board_can_transmit_ext_txfifo_nonblocking(uint32_t ext_id, void *data, uint8_t dlc)
{
    mcan_tx_frame_t can_frame = {};
    uint32_t fifo_index = 0;
    can_frame.dlc = dlc;
    can_frame.ext_id = ext_id;
    can_frame.use_ext_id = true;
    can_frame.event_fifo_control = 1;
    can_frame.message_marker_l = 0;
    memcpy(can_frame.data_8, data, dlc);

    int ret = mcan_transmit_via_txfifo_nonblocking(BOARD_APP_CAN_BASE, &can_frame, &fifo_index);
    if (ret != status_success)
        return -ret;
    return fifo_index;
}