#include "board.h"
#include "eeprom_emulation.h"

static e2p_t e2p;

static uint32_t config_read(uint8_t *buf, uint32_t addr, uint32_t size)
{
    return nor_flash_read(&e2p.nor_config, buf, addr, size);
}

static uint32_t config_write(uint8_t *buf, uint32_t addr, uint32_t size)
{
    return nor_flash_write(&e2p.nor_config, buf, addr, size);
}

static void config_erase(uint32_t start_addr, uint32_t size)
{
    nor_flash_erase(&e2p.nor_config, start_addr, size);
}

void config_init(void)
{
    e2p.nor_config.xpi_base = BOARD_APP_XPI_NOR_XPI_BASE;
    e2p.nor_config.opt_header = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
    e2p.nor_config.opt0 = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
    e2p.nor_config.opt1 = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;
    e2p.nor_config.base_addr = 0x80000000;
    e2p.config.start_addr = 0x80080000;
    e2p.config.erase_size = 4096;
    e2p.config.sector_cnt = 128;
    e2p.config.version = 0x0001;
    e2p.config.flash_read = config_read;
    e2p.config.flash_write = config_write;
    e2p.config.flash_erase = config_erase;

    nor_flash_init(&e2p.nor_config);
    e2p_config(&e2p);
}

int config_set(const char *name, void *buf, size_t len)
{
    uint32_t block_id = e2p_generate_id(name);
    return e2p_write(&e2p, block_id, len, buf);
}

int config_get(const char *name, void *buf, size_t len)
{
    uint32_t block_id = e2p_generate_id(name);
    return e2p_read(&e2p, block_id, len, buf);
}