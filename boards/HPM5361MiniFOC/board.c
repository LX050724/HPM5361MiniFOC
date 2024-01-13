/*
 * Copyright (c) 2023 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "board.h"
#include "SEGGER_RTT.h"
#include "pinmux.h"
#include "hpm_sdk_version.h"
#include "hpm_gptmr_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_usb_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_i2c_drv.h"
#include "hpm_pcfg_drv.h"

static board_timer_cb timer_cb;

// #define SEGGER_RTT_printf(0, fmt, ...)

/**
 * @brief FLASH configuration option definitions:
 * option[0]:
 *    [31:16] 0xfcf9 - FLASH configuration option tag
 *    [15:4]  0 - Reserved
 *    [3:0]   option words (exclude option[0])
 * option[1]:
 *    [31:28] Flash probe type
 *      0 - SFDP SDR / 1 - SFDP DDR
 *      2 - 1-4-4 Read (0xEB, 24-bit address) / 3 - 1-2-2 Read(0xBB, 24-bit address)
 *      4 - HyperFLASH 1.8V / 5 - HyperFLASH 3V
 *      6 - OctaBus DDR (SPI -> OPI DDR)
 *      8 - Xccela DDR (SPI -> OPI DDR)
 *      10 - EcoXiP DDR (SPI -> OPI DDR)
 *    [27:24] Command Pads after Power-on Reset
 *      0 - SPI / 1 - DPI / 2 - QPI / 3 - OPI
 *    [23:20] Command Pads after Configuring FLASH
 *      0 - SPI / 1 - DPI / 2 - QPI / 3 - OPI
 *    [19:16] Quad Enable Sequence (for the device support SFDP 1.0 only)
 *      0 - Not needed
 *      1 - QE bit is at bit 6 in Status Register 1
 *      2 - QE bit is at bit1 in Status Register 2
 *      3 - QE bit is at bit7 in Status Register 2
 *      4 - QE bit is at bit1 in Status Register 2 and should be programmed by 0x31
 *    [15:8] Dummy cycles
 *      0 - Auto-probed / detected / default value
 *      Others - User specified value, for DDR read, the dummy cycles should be 2 * cycles on FLASH datasheet
 *    [7:4] Misc.
 *      0 - Not used
 *      1 - SPI mode
 *      2 - Internal loopback
 *      3 - External DQS
 *    [3:0] Frequency option
 *      1 - 30MHz / 2 - 50MHz / 3 - 66MHz / 4 - 80MHz / 5 - 100MHz / 6 - 120MHz / 7 - 133MHz / 8 - 166MHz
 *
 * option[2] (Effective only if the bit[3:0] in option[0] > 1)
 *    [31:20]  Reserved
 *    [19:16] IO voltage
 *      0 - 3V / 1 - 1.8V
 *    [15:12] Pin group
 *      0 - 1st group / 1 - 2nd group
 *    [11:8] Connection selection
 *      0 - CA_CS0 / 1 - CB_CS0 / 2 - CA_CS0 + CB_CS0 (Two FLASH connected to CA and CB respectively)
 *    [7:0] Drive Strength
 *      0 - Default value
 * option[3] (Effective only if the bit[3:0] in option[0] > 2, required only for the QSPI NOR FLASH that not supports
 *              JESD216)
 *    [31:16] reserved
 *    [15:12] Sector Erase Command Option, not required here
 *    [11:8]  Sector Size Option, not required here
 *    [7:0] Flash Size Option
 *      0 - 4MB / 1 - 8MB / 2 - 16MB
 */
#if defined(FLASH_XIP) && FLASH_XIP
__attribute__ ((section(".nor_cfg_option"))) const uint32_t option[4] = {0xfcf90002, 0x00000006, 0x1000, 0x0};
#endif

#if defined(FLASH_UF2) && FLASH_UF2
ATTR_PLACE_AT(".uf2_signature") const uint32_t uf2_signature = BOARD_UF2_SIGNATURE;
#endif


void board_print_banner(void)
{
    const uint8_t banner[] = "\n"
"----------------------------------------------------------------------\n"
"$$\\   $$\\ $$$$$$$\\  $$\\      $$\\ $$\\\n"
"$$ |  $$ |$$  __$$\\ $$$\\    $$$ |\\__|\n"
"$$ |  $$ |$$ |  $$ |$$$$\\  $$$$ |$$\\  $$$$$$$\\  $$$$$$\\   $$$$$$\\\n"
"$$$$$$$$ |$$$$$$$  |$$\\$$\\$$ $$ |$$ |$$  _____|$$  __$$\\ $$  __$$\\\n"
"$$  __$$ |$$  ____/ $$ \\$$$  $$ |$$ |$$ /      $$ |  \\__|$$ /  $$ |\n"
"$$ |  $$ |$$ |      $$ |\\$  /$$ |$$ |$$ |      $$ |      $$ |  $$ |\n"
"$$ |  $$ |$$ |      $$ | \\_/ $$ |$$ |\\$$$$$$$\\ $$ |      \\$$$$$$  |\n"
"\\__|  \\__|\\__|      \\__|     \\__|\\__| \\_______|\\__|       \\______/\n"
"----------------------------------------------------------------------\n";
#ifdef SDK_VERSION_STRING
    SEGGER_RTT_printf(0, "hpm_sdk: %s\n", SDK_VERSION_STRING);
#endif
    SEGGER_RTT_printf(0, "%s", banner);
}

void board_print_clock_freq(void)
{
    SEGGER_RTT_printf(0, "==============================\n");
    SEGGER_RTT_printf(0, " %s clock summary\n", BOARD_NAME);
    SEGGER_RTT_printf(0, "==============================\n");
    SEGGER_RTT_printf(0, "cpu0:\t\t %luHz\n", clock_get_frequency(clock_cpu0));
    SEGGER_RTT_printf(0, "ahb:\t\t %luHz\n", clock_get_frequency(clock_ahb));
    SEGGER_RTT_printf(0, "mchtmr0:\t %luHz\n", clock_get_frequency(clock_mchtmr0));
    SEGGER_RTT_printf(0, "xpi0:\t\t %luHz\n", clock_get_frequency(clock_xpi0));
    SEGGER_RTT_printf(0, "==============================\n");
}

void board_init(void)
{
    init_jtag_pins();
    init_xtal_pins();
    SEGGER_RTT_Init();
    init_py_pins_as_pgpio();
    board_init_usb_dp_dm_pins();

    board_init_clock();
    board_init_pmp();
#if BOARD_SHOW_CLOCK
    board_print_clock_freq();
#endif
#if BOARD_SHOW_BANNER
    board_print_banner();
#endif
}

void board_init_usb_dp_dm_pins(void)
{
    /* Disconnect usb dp/dm pins pull down 45ohm resistance */

    while (sysctl_resource_any_is_busy(HPM_SYSCTL)) {
        ;
    }
    if (pllctlv2_xtal_is_stable(HPM_PLLCTLV2) && pllctlv2_xtal_is_enabled(HPM_PLLCTLV2)) {
        if (clock_check_in_group(clock_usb0, 0)) {
            usb_phy_disable_dp_dm_pulldown(HPM_USB0);
        } else {
            clock_add_to_group(clock_usb0, 0);
            usb_phy_disable_dp_dm_pulldown(HPM_USB0);
            clock_remove_from_group(clock_usb0, 0);
        }
    } else {
        uint8_t tmp;
        tmp = sysctl_resource_target_get_mode(HPM_SYSCTL, sysctl_resource_xtal);
        sysctl_resource_target_set_mode(HPM_SYSCTL, sysctl_resource_xtal, 0x03);
        clock_add_to_group(clock_usb0, 0);
        usb_phy_disable_dp_dm_pulldown(HPM_USB0);
        clock_remove_from_group(clock_usb0, 0);
        while (sysctl_resource_target_is_busy(HPM_SYSCTL, sysctl_resource_usb0)) {
            ;
        }
        sysctl_resource_target_set_mode(HPM_SYSCTL, sysctl_resource_xtal, tmp);
    }
}

void board_init_clock(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);

    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }

    /* group0[0] */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_rom, 0);
    clock_add_to_group(clock_can0, 0);
    clock_add_to_group(clock_can1, 0);
    clock_add_to_group(clock_can2, 0);
    clock_add_to_group(clock_can3, 0);
    clock_add_to_group(clock_ptpc, 0);
    clock_add_to_group(clock_lin0, 0);
    clock_add_to_group(clock_lin1, 0);
    clock_add_to_group(clock_lin2, 0);
    clock_add_to_group(clock_lin3, 0);
    clock_add_to_group(clock_gptmr0, 0);
    clock_add_to_group(clock_gptmr1, 0);
    clock_add_to_group(clock_gptmr2, 0);
    clock_add_to_group(clock_gptmr3, 0);
    clock_add_to_group(clock_i2c0, 0);
    clock_add_to_group(clock_i2c1, 0);
    clock_add_to_group(clock_i2c2, 0);
    clock_add_to_group(clock_i2c3, 0);
    clock_add_to_group(clock_spi0, 0);
    clock_add_to_group(clock_spi1, 0);
    clock_add_to_group(clock_spi2, 0);
    clock_add_to_group(clock_spi3, 0);
    clock_add_to_group(clock_uart0, 0);
    clock_add_to_group(clock_uart1, 0);
    clock_add_to_group(clock_uart2, 0);
    clock_add_to_group(clock_uart3, 0);
    clock_add_to_group(clock_uart4, 0);
    clock_add_to_group(clock_uart5, 0);
    clock_add_to_group(clock_uart6, 0);
    /* group0[1] */
    clock_add_to_group(clock_uart7, 0);
    clock_add_to_group(clock_watchdog0, 0);
    clock_add_to_group(clock_watchdog1, 0);
    clock_add_to_group(clock_mbx0, 0);
    clock_add_to_group(clock_tsns, 0);
    clock_add_to_group(clock_crc0, 0);
    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_adc1, 0);
    clock_add_to_group(clock_dac0, 0);
    clock_add_to_group(clock_dac1, 0);
    clock_add_to_group(clock_acmp, 0);
    clock_add_to_group(clock_opa0, 0);
    clock_add_to_group(clock_opa1, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_rng, 0);
    clock_add_to_group(clock_sdp, 0);
    clock_add_to_group(clock_kman, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_usb0, 0);

    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Bump up DCDC voltage to 1175mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1175);

#if 1
    /* Configure CPU to 480MHz, AXI/AHB to 320MHz */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll0_clk0, 2, 2);
    /* Configure PLL0 Post Divider */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 0, 0);    /* PLL0CLK0: 960MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 1, 3);    /* PLL0CLK1: 600MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 2, 7);    /* PLL0CLK2: 400MHz */
    /* Configure PLL0 Frequency to 960MHz */
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, 0, 960000000);
#else
    /* Configure PLL0 Post Divider */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 0, 0);    /* PLL0CLK0: 650MHz     */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 1, 3);    /* PLL0CLK1: 406.25MHz  */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 2, 7);    /* PLL0CLK2: 270.833MHz */
    /* Configure PLL0 Frequency to 650MHz */
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, 0, 600000000);
    /* Configure CPU to 650MHz, AXI/AHB to 325MHz */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll0_clk0, 1, 2);
#endif

    clock_update_core_clock();

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

void board_delay_us(uint32_t us)
{
    clock_cpu_delay_us(us);
}

void board_delay_ms(uint32_t ms)
{
    clock_cpu_delay_ms(ms);
}

SDK_DECLARE_EXT_ISR_M(BOARD_CALLBACK_TIMER_IRQ, board_timer_isr);
void board_timer_isr(void)
{
    if (gptmr_check_status(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_STAT_MASK(BOARD_CALLBACK_TIMER_CH))) {
        gptmr_clear_status(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_STAT_MASK(BOARD_CALLBACK_TIMER_CH));
        timer_cb();
    }
}

void board_timer_create(uint32_t ms, board_timer_cb cb)
{
    uint32_t gptmr_freq;
    gptmr_channel_config_t config;

    timer_cb = cb;
    gptmr_channel_get_default_config(BOARD_CALLBACK_TIMER, &config);

    clock_add_to_group(BOARD_CALLBACK_TIMER_CLK_NAME, 0);
    gptmr_freq = clock_get_frequency(BOARD_CALLBACK_TIMER_CLK_NAME);

    config.reload = gptmr_freq / 1000 * ms;
    gptmr_channel_config(BOARD_CALLBACK_TIMER, BOARD_CALLBACK_TIMER_CH, &config, false);
    gptmr_enable_irq(BOARD_CALLBACK_TIMER, GPTMR_CH_RLD_IRQ_MASK(BOARD_CALLBACK_TIMER_CH));
    intc_m_enable_irq_with_priority(BOARD_CALLBACK_TIMER_IRQ, 1);

    gptmr_start_counter(BOARD_CALLBACK_TIMER, BOARD_CALLBACK_TIMER_CH);
}

void board_init_gpio_pins(void)
{
    init_gpio_pins();
    gpio_set_pin_input(BOARD_APP_GPIO_CTRL, BOARD_APP_GPIO_INDEX, BOARD_APP_GPIO_PIN);
}

void board_init_led_pins(void)
{
    init_led_pins_as_gpio();
    gpio_set_pin_output_with_initial(BOARD_LED_GPIO_CTRL, BOARD_LED_GPIO_INDEX, BOARD_LED_GPIO_GREEN_PIN, board_get_led_gpio_off_level());
    gpio_set_pin_output_with_initial(BOARD_LED_GPIO_CTRL, BOARD_LED_GPIO_INDEX, BOARD_LED_GPIO_RED_PIN, board_get_led_gpio_off_level());
}

void board_init_usb_pins(void)
{
    init_usb_pins();
    usb_hcd_set_power_ctrl_polarity(BOARD_USB, true);

    /* As QFN32, QFN48 and LQFP64 has no vbus pin, so should be call usb_phy_using_internal_vbus() API to use internal vbus. */
    usb_phy_using_internal_vbus(BOARD_USB);
}

void board_led_write(uint8_t pin, uint8_t state)
{
    gpio_write_pin(BOARD_LED_GPIO_CTRL, BOARD_LED_GPIO_INDEX, pin, state);
}

void board_led_toggle(uint8_t pin)
{
    gpio_toggle_pin(BOARD_LED_GPIO_CTRL, BOARD_LED_GPIO_INDEX, pin);
}

void board_init_uart(UART_Type *ptr)
{
    /* configure uart's pin before opening uart's clock */
    init_uart_pins(ptr);
    board_init_uart_clock(ptr);
}

void board_ungate_mchtmr_at_lp_mode(void)
{
    /* Keep cpu clock on wfi, so that mchtmr irq can still work after wfi */
    sysctl_set_cpu_lp_mode(HPM_SYSCTL, BOARD_RUNNING_CORE, cpu_lp_mode_ungate_cpu_clock);
}

uint32_t board_init_spi_clock(SPI_Type *ptr)
{
    if (ptr == HPM_SPI1) {
        clock_add_to_group(clock_spi1, 0);
        return clock_get_frequency(clock_spi1);
    }
    return 0;
}

void board_init_spi_pins(SPI_Type *ptr)
{
    init_spi_pins(ptr);
}

void board_write_spi_cs(uint32_t pin, uint8_t state)
{
    gpio_write_pin(BOARD_SPI_CS_GPIO_CTRL, GPIO_GET_PORT_INDEX(pin), GPIO_GET_PIN_INDEX(pin), state);
}

void board_init_spi_pins_with_gpio_as_cs(SPI_Type *ptr)
{
    init_spi_pins_with_gpio_as_cs(ptr);
    gpio_set_pin_output_with_initial(BOARD_SPI_CS_GPIO_CTRL, GPIO_GET_PORT_INDEX(BOARD_SPI_CS_PIN),
                                     GPIO_GET_PIN_INDEX(BOARD_SPI_CS_PIN), !BOARD_SPI_CS_ACTIVE_LEVEL);
}

void board_init_lin_pins(LINV2_Type *ptr)
{
    init_lin_pins(ptr);
    //gpio_set_pin_output_with_initial(BOARD_12V_EN_GPIO_CTRL, BOARD_12V_EN_GPIO_INDEX, BOARD_12V_EN_GPIO_PIN, 1);    /* enable 12v output */
}

uint32_t board_init_lin_clock(LINV2_Type *ptr)
{
    if (ptr == HPM_LIN2) {
        clock_add_to_group(clock_lin2, 0);
        clock_set_source_divider(clock_lin2, clk_src_pll1_clk0, 40U); /* 20MHz */
        return clock_get_frequency(clock_lin2);
    } else if (ptr == HPM_LIN3) {
        clock_add_to_group(clock_lin3, 0);
        clock_set_source_divider(clock_lin3, clk_src_pll1_clk0, 40U); /* 20MHz */
        return clock_get_frequency(clock_lin3);
    }
    return 0;
}

void board_usb_vbus_ctrl(uint8_t usb_index, uint8_t level)
{
}

uint32_t board_init_adc16_clock(ADC16_Type *ptr, bool clk_src_ahb)
{
    uint32_t freq = 0;

    if (ptr == HPM_ADC0) {
        if (clk_src_ahb) {
            /* Configure the ADC clock from AHB (@200MHz by default)*/
            clock_set_adc_source(clock_adc0, clk_adc_src_ahb0);
        } else {
            /* Configure the ADC clock from pll0_clk0 divided by 2 (@200MHz by default) */
            clock_set_adc_source(clock_adc0, clk_adc_src_ana0);
            clock_set_source_divider(clock_ana0, clk_src_pll0_clk2, 2U);
        }

        freq = clock_get_frequency(clock_adc0);
    } else if (ptr == HPM_ADC1) {
        if (clk_src_ahb) {
            /* Configure the ADC clock from AHB (@200MHz by default)*/
            clock_set_adc_source(clock_adc1, clk_adc_src_ahb0);
        } else {
            /* Configure the ADC clock from pll0_clk0 divided by 2 (@200MHz by default) */
            clock_set_adc_source(clock_adc1, clk_adc_src_ana1);
            clock_set_source_divider(clock_ana1, clk_src_pll0_clk2, 2U);
        }

        freq = clock_get_frequency(clock_adc1);
    }

    return freq;
}

void board_init_adc16_pins(void)
{
    init_adc_pins();
}

uint32_t board_init_dac_clock(DAC_Type *ptr, bool clk_src_ahb)
{
    uint32_t freq = 0;

    if (ptr == HPM_DAC0) {
        if (clk_src_ahb == true) {
            /* Configure the DAC clock to 180MHz */
            clock_set_dac_source(clock_dac0, clk_dac_src_ahb0);
        } else {
            /* Configure the DAC clock to 166MHz */
            clock_set_dac_source(clock_dac0, clk_dac_src_ana2);
            clock_set_source_divider(clock_ana2, clk_src_pll0_clk1, 2);
        }

        freq = clock_get_frequency(clock_dac0);
    } else if (ptr == HPM_DAC1) {
        if (clk_src_ahb == true) {
            /* Configure the DAC clock to 180MHz */
            clock_set_dac_source(clock_dac1, clk_dac_src_ahb0);
        } else {
            /* Configure the DAC clock to 166MHz */
            clock_set_dac_source(clock_dac1, clk_dac_src_ana3);
            clock_set_source_divider(clock_ana3, clk_src_pll0_clk1, 2);
        }

        freq = clock_get_frequency(clock_dac1);
    }

    return freq;
}

void board_init_can_pins(MCAN_Type *ptr)
{
    init_can_pins(ptr);
}

uint32_t board_init_can_clock(MCAN_Type *ptr)
{
    uint32_t freq = 0;
    if (ptr == HPM_MCAN0) {
        clock_add_to_group(clock_can0, 0);
        clock_set_source_divider(clock_can0, clk_src_pll1_clk0, 10);
        freq = clock_get_frequency(clock_can0);
    }
    if (ptr == HPM_MCAN1) {
        clock_add_to_group(clock_can1, 0);
        clock_set_source_divider(clock_can1, clk_src_pll1_clk0, 10);
        freq = clock_get_frequency(clock_can1);
    }
    if (ptr == HPM_MCAN2) {
        clock_add_to_group(clock_can2, 0);
        clock_set_source_divider(clock_can2, clk_src_pll1_clk0, 10);
        freq = clock_get_frequency(clock_can2);
    }
    if (ptr == HPM_MCAN3) {
        clock_add_to_group(clock_can3, 0);
        clock_set_source_divider(clock_can3, clk_src_pll1_clk0, 10);
        freq = clock_get_frequency(clock_can3);
    }
    return freq;
}

void board_init_rgb_pwm_pins(void)
{
    init_led_pins_as_pwm();
}

void board_disable_output_rgb_led(uint8_t color)
{
}

void board_enable_output_rgb_led(uint8_t color)
{
}

void board_init_dac_pins(DAC_Type *ptr)
{
    init_dac_pins(ptr);
}

uint8_t board_get_led_pwm_off_level(void)
{
    return BOARD_LED_OFF_LEVEL;
}

uint8_t board_get_led_gpio_off_level(void)
{
    return BOARD_LED_OFF_LEVEL;
}

void board_init_pmp(void)
{
}

uint32_t board_init_uart_clock(UART_Type *ptr)
{
    uint32_t freq = 0U;
    if (ptr == HPM_UART0) {
        clock_set_source_divider(clock_uart0, clk_src_osc24m, 1);
        clock_add_to_group(clock_uart0, 0);
        freq = clock_get_frequency(clock_uart0);
    } else if (ptr == HPM_UART1) {
        clock_set_source_divider(clock_uart1, clk_src_osc24m, 1);
        clock_add_to_group(clock_uart1, 0);
        freq = clock_get_frequency(clock_uart1);
    } else if (ptr == HPM_UART2) {
        clock_set_source_divider(clock_uart2, clk_src_pll0_clk2, 8);
        clock_add_to_group(clock_uart2, 0);
        freq = clock_get_frequency(clock_uart2);
    }

    return freq;
}

void board_init_sei_pins(SEI_Type *ptr, uint8_t sei_ctrl_idx)
{
    init_sei_pins(ptr, sei_ctrl_idx);
}

void board_i2c_bus_clear(I2C_Type *ptr)
{
    if (i2c_get_line_scl_status(ptr) == false) {
        SEGGER_RTT_printf(0, "CLK is low, please power cycle the board\n");
        while (1) {
        }
    }
    if (i2c_get_line_sda_status(ptr) == false) {
        SEGGER_RTT_printf(0, "SDA is low, try to issue I2C bus clear\n");
    } else {
        SEGGER_RTT_printf(0, "I2C bus is ready\n");
        return;
    }
    i2s_gen_reset_signal(ptr, 9);
    board_delay_ms(100);
    SEGGER_RTT_printf(0, "I2C bus is cleared\n");
}

void board_init_i2c(I2C_Type *ptr)
{
    i2c_config_t config;
    hpm_stat_t stat;
    uint32_t freq;
    if (ptr == NULL) {
        return;
    }
    init_i2c_pins(ptr);
    board_i2c_bus_clear(ptr);

    clock_add_to_group(clock_i2c0, 0);
    clock_add_to_group(clock_i2c1, 0);
    clock_add_to_group(clock_i2c2, 0);
    clock_add_to_group(clock_i2c3, 0);
    /* Configure the I2C clock to 24MHz */
    clock_set_source_divider(BOARD_APP_I2C_CLK_NAME, clk_src_osc24m, 1U);

    config.i2c_mode = i2c_mode_normal;
    config.is_10bit_addressing = false;
    freq = clock_get_frequency(BOARD_APP_I2C_CLK_NAME);
    stat = i2c_init_master(ptr, freq, &config);
    if (stat != status_success) {
        SEGGER_RTT_printf(0, "failed to initialize i2c 0x%lx\n", (uint32_t) ptr);
        while (1) {
        }
    }

}

