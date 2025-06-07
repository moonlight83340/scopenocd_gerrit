// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 * Copyright (C) 2025 Space Cubics Inc                                     *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/image.h>

// Configuration Memory Register
#define SCOBCA1_FPGA_QSPI_CFG_BASE 0x40000000

// Offset
#define SCQSPI_ACR_OFFSET    0x0000	// QSPI Access Control Register
#define SCQSPI_TDR_OFFSET    0x0004	// QSPI TX Data Register
#define SCQSPI_RDR_OFFSET    0x0008	// QSPI RX Data Register
#define SCQSPI_ASR_OFFSET    0x000C	// QSPI Access Status Register
#define SCQSPI_FIFOSR_OFFSET 0x0010	// QSPI FIFO Status Register
#define SCQSPI_FIFORR_OFFSET 0x0014	// QSPI FIFO Reset Register
#define SCQSPI_ISR_OFFSET    0x0020	// QSPI Interrupt Status Register
#define SCQSPI_IER_OFFSET    0x0024	// QSPI Interrupt Enable Register
#define SCQSPI_CCR_OFFSET    0x0030	// QSPI Clock Control Register
#define SCQSPI_DCMSR_OFFSET  0x0034	// QSPI Data Capture Mode Setting Register
#define SCQSPI_FTLSR_OFFSET  0x0038	// QSPI FIFO Threshold Level Setting Register
#define SCQSPI_VER_OFFSET    0xF000	// QSPI Controller IP Version Register

// QSPI Control Register for Configuration Memory
#define SCOBCA1_FPGA_NORFLASH_QSPI_ACR    (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_ACR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_TDR    (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_TDR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_RDR    (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_RDR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_ASR    (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_ASR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FIFOSR (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_FIFOSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FIFORR (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_FIFORR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_ISR    (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_ISR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_IER    (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_IER_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_CCR    (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_CCR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_DCMSR  (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_DCMSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_FTLSR  (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_FTLSR_OFFSET)
#define SCOBCA1_FPGA_NORFLASH_QSPI_VER    (SCOBCA1_FPGA_QSPI_CFG_BASE + SCQSPI_VER_OFFSET)

#define SCOBCA1_SYSREG_CFGMEMSEL(x) ((((x) & BIT(0)) << 4))
#define SCOBCA1_SYSREG_CFGMEMMON(x) ((((x) & BIT(0)) << 5))
#define SCOBCA1_SYSREG_CFGMEMCTL    0x4F000010

#define SCQSPI_DATA_MEM0_SS          0x01
#define SCQSPI_CFG_MEM0              0U
#define SCQSPI_CFG_MEM1              1U
#define SCQSPI_ASR_IDLE              0x00
#define SCQSPI_ASR_BUSY              0x01
#define SCQSPI_RX_FIFO_MAX_BYTE      16U
#define SCQSPI_TX_FIFO_MAX_BYTE      16U
#define SCQSPI_SPI_MODE_QUAD         0x00020000
#define SCQSPI_ERASE_BLOCK_WAIT_MS   800U
#define SCQSPI_ERASE_SECTOR_WAIT_MS  10U
#define SCQSPI_PAGE_BUFFER_BYTE      256U
#define SCQSPI_REG_READ_RETRY(count) (count)
#define SCQSPI_CRC32_INIT            0xFFFFFFFF
#define SCQSPI_CRC32_FINAL(crc)      (~(crc))
#define SCQSPI_VERIFY_CHUNK_SIZE     4096U
#define SCQSPI_ALL_MASK              GENMASK(31, 0)

#define SYSREG_CFGMEMSEL_MON_MASK GENMASK(5, 4)	// CFGMEMSEL and CFGMEMMON bits mask

struct scqspi_flash_bank {
	struct target *target;
	bool probed;
	uint32_t flash_addr;
	uint8_t spi_ss;
	uint8_t mem_no;
	struct flash_device dev;
};

/* -------------------------------------------------------------------------
 * Internal helper functions for scqspi flash driver implementation
 * -------------------------------------------------------------------------*/

static bool verify(struct target *target, uint32_t addr, uint32_t exp, uint32_t read_mask,
	unsigned int retry)
{
	uint32_t val;
	int ret;

	for (unsigned int i = 0; i <= retry; i++) {
		ret = target_read_u32(target, addr, &val);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to read the register value: %d", ret);
			break;
		}
		val &= read_mask;

		if (val == exp) {
			LOG_DEBUG("read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)", addr, val, exp,
				i + 1);
			return true;
		} else if (i + 1 == retry) {
			LOG_ERROR("read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)", addr, val, exp,
				i + 1);
		} else {
			LOG_DEBUG("read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)", addr, val, exp,
				i + 1);
		}
		usleep(1);
	}

	LOG_ERROR("Verification failed: retry count: %d", retry);
	return false;
}

static bool is_qspi_control_done(struct target *target)
{
	int ret;

	LOG_DEBUG("Confirm QSPI Interrupt Status is `SPI Control Done`");

	if (!verify(target, SCOBCA1_FPGA_NORFLASH_QSPI_ISR, 0x01, SCQSPI_ALL_MASK,
			SCQSPI_REG_READ_RETRY(10))) {
		LOG_ERROR("Confirm QSPI Interrupt Status failed");
		return false;
	}

	LOG_DEBUG("Clear QSPI Interrupt Status");
	ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ISR, 0x01);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read QSPI Interrupt Status: %d", ret);
		return false;
	}

	if (!verify(target, SCOBCA1_FPGA_NORFLASH_QSPI_ISR, 0x00, SCQSPI_ALL_MASK,
			SCQSPI_REG_READ_RETRY(10))) {
		LOG_ERROR("Failed to read QSPI Interrupt Status");
		return false;
	}

	return true;
}

static bool is_qspi_idle(struct target *target)
{
	LOG_DEBUG("Confirm QSPI Access Status is `Idle`");

	if (!verify(target, SCOBCA1_FPGA_NORFLASH_QSPI_ASR, SCQSPI_ASR_IDLE, SCQSPI_ALL_MASK,
			SCQSPI_REG_READ_RETRY(10))) {
		LOG_ERROR("QSPI (Config Memory) is busy");
		return false;
	}

	return true;
}

static int activate_spi_ss(struct target *target, uint32_t spi_mode)
{
	LOG_DEBUG("Activate SPI SS with %08x", spi_mode);

	int ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ACR, spi_mode);
	if (ret != ERROR_OK)
		return ret;

	if (!is_qspi_idle(target))
		return ERROR_FLASH_BUSY;

	return ERROR_OK;
}

static int inactivate_spi_ss(struct target *target)
{
	LOG_DEBUG("Inactivate SPI SS");

	int ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ACR, 0x00);
	if (ret != ERROR_OK)
		return ret;

	if (!is_qspi_idle(target))
		return ERROR_FLASH_BUSY;

	return ERROR_OK;
}

static int reset_qspi_rxfifo(struct target *target)
{
	return target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_FIFORR, 0x01);
}

static int clear_status_register(struct target *target, uint32_t spi_ss)
{
	int ret;

	ret = activate_spi_ss(target, spi_ss);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to activate SPI SS");
		goto end;
	}

	// Clear All ISR
	ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_ISR, 0xFFFFFFFF);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to clear ALL ISR : %d", ret);
		goto inactive_ss;
	}

	ret = reset_qspi_rxfifo(target);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to reset QSPI FIFO : %d", ret);
		goto inactive_ss;
	}

	LOG_DEBUG("Clear Status Register (Instructure:0x30) ");
	ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR, 0x30);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to clear Status Register : %d", ret);
		goto inactive_ss;
	}

inactive_ss:
	ret = inactivate_spi_ss(target);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to inactivate SPI SS");
		goto end;
	}

	if (!is_qspi_control_done(target)) {
		LOG_ERROR("Confirm SPI Control is Done failed");
		ret = ERROR_FAIL;
	}

end:
	return ret;
}

static int select_mem(struct target *target, uint8_t mem_no)
{
	int ret;
	uint32_t expval = SCOBCA1_SYSREG_CFGMEMSEL(mem_no) | SCOBCA1_SYSREG_CFGMEMMON(mem_no);

	LOG_DEBUG("Select Config Memory %d", mem_no);

	ret = target_write_u32(target, SCOBCA1_SYSREG_CFGMEMCTL, SCOBCA1_SYSREG_CFGMEMSEL(mem_no));
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write Config Memory %d", mem_no);
		return ret;
	}

	if (!verify(target, SCOBCA1_SYSREG_CFGMEMCTL, expval, SYSREG_CFGMEMSEL_MON_MASK,
			SCQSPI_REG_READ_RETRY(1000))) {
		LOG_ERROR("Failed to select Config Memory %d", mem_no);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int scqspi_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;
	int ret;

	ret = select_mem(target, scqspi_info->mem_no);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to select memory");
		return ret;
	}

	LOG_DEBUG("Clear Status Register");
	ret = clear_status_register(target, scqspi_info->spi_ss);
	if (ret != ERROR_OK)
		LOG_ERROR("Failed to clear Status register");

	return ret;
}

static int scqspi_read_id(struct flash_bank *bank, uint32_t *id)
{
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;
	struct target *target = bank->target;
	uint8_t din[3] = {0, 0, 0};
	uint8_t rdata;
	int ret;

	ret = activate_spi_ss(target, scqspi_info->spi_ss);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to activate SPI SS");
		goto end;
	}

	ret = reset_qspi_rxfifo(target);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to reset QSPI FIFO");
		goto inactivate_ss;
	}

	LOG_DEBUG("Send SPIFLASH_READ_ID cmd");
	ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR, SPIFLASH_READ_ID);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write SPIFLASH_READ_ID command");
		goto inactivate_ss;
	}

	if (!is_qspi_idle(target)) {
		ret = ERROR_FLASH_BUSY;
		goto inactivate_ss;
	}

	LOG_DEBUG("Read SPIFLASH_READ_ID");

	// Request RX data
	for (size_t i = 0; i < sizeof(din); i++) {
		ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_RDR, 0x00);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to request RX data");
			goto inactivate_ss;
		}
	}

	if (!is_qspi_idle(target)) {
		ret = ERROR_FLASH_BUSY;
		goto inactivate_ss;
	}

	// Read RX FIFO
	for (size_t i = 0; i < sizeof(din); i++) {
		ret = target_read_u8(target, SCOBCA1_FPGA_NORFLASH_QSPI_RDR, &rdata);
		if (ret != ERROR_OK)
			goto inactivate_ss;

		din[i] = rdata;
		LOG_DEBUG("Read byte %zu: 0x%02x", i, din[i]);
	}

	*id = (din[2] << 16) | (din[1] << 8) | din[0];

inactivate_ss:
	ret = inactivate_spi_ss(target);
	if (ret != ERROR_OK)
		LOG_ERROR("Failed to inactivate SPI SS");

end:
	return ret;
}

static int read_and_verify_rx_data(struct target *target, size_t exp_size, uint32_t *exp_val)
{
	int ret;
	uint32_t rdata;

	ret = reset_qspi_rxfifo(target);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to reset QSPI FIFO");
		goto end;
	}

	LOG_DEBUG("Request RX FIFO %zu byte", exp_size);
	for (size_t i = 0; i < exp_size; i++) {
		ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_RDR, 0x00);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to resquest QSPI Data: %d", ret);
			goto end;
		}
	}

	if (!is_qspi_idle(target)) {
		ret = ERROR_FLASH_BUSY;
		goto end;
	}

	LOG_DEBUG("Read RX FIFO %zu byte and verify the value", exp_size);
	for (size_t i = 0; i < exp_size; i++) {
		ret = target_read_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_RDR, &rdata);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to read RX FIFO %zu byte", exp_size);
			goto end;
		}

		if (exp_val[i] != rdata) {
			LOG_ERROR("Read RX FIFO %zu byte failed: expected 0x%08x, got 0x%08x", i,
				exp_val[i], rdata);
			ret = ERROR_FAIL;
		}
	}

end:
	return ret;
}

static int verify_status_register1(struct target *target, uint32_t spi_ss, size_t exp_size,
	uint32_t *exp_val)
{
	int ret;

	ret = activate_spi_ss(target, spi_ss);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to activate SPI SS : %d", ret);
		goto end;
	}

	LOG_DEBUG("Request Status Register");
	ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR, SPIFLASH_READ_STATUS);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write SPIFLASH_READ_STATUS command : %d", ret);
		goto inactivate_ss;
	}

	if (!is_qspi_idle(target)) {
		ret = ERROR_FLASH_BUSY;
		goto inactivate_ss;
	}

	// Read RX data (2byte) and Verify
	ret = read_and_verify_rx_data(target, exp_size, exp_val);
	if (ret != ERROR_OK)
		LOG_ERROR("Failed to read and verify RX data : %d", ret);

inactivate_ss:
	ret = inactivate_spi_ss(target);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to inactivate SPI SS : %d", ret);
		goto end;
	}

	if (!is_qspi_control_done(target)) {
		LOG_ERROR("Confirm SPI Control is Done failed");
		ret = ERROR_FAIL;
	}

end:
	return ret;
}

static bool is_write_enable(struct target *target, uint32_t spi_ss)
{
	LOG_DEBUG("Verify write enable");

	int ret;
	uint32_t exp_write_enable[] = {0x02, 0x02};

	ret = verify_status_register1(target, spi_ss, ARRAY_SIZE(exp_write_enable),
			exp_write_enable);
	if (ret != ERROR_OK) {
		LOG_ERROR("Write enable not set");
		return false;
	}

	return true;
}

static bool is_write_disable(struct target *target, uint32_t spi_ss)
{
	int ret;
	uint32_t exp_write_disable[] = {0x00, 0x00};

	LOG_DEBUG("Verify write disable");

	ret = verify_status_register1(target, spi_ss, ARRAY_SIZE(exp_write_disable),
			exp_write_disable);
	if (ret != ERROR_OK) {
		LOG_ERROR("Write disable not set");
		return false;
	}

	return true;
}

static int set_write_enable(struct target *target, uint32_t spi_ss)
{
	int ret;

	ret = activate_spi_ss(target, spi_ss);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to activate SPI SS");
		goto end;
	}

	LOG_DEBUG("Set Write Enable");
	ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR, SPIFLASH_WRITE_ENABLE);
	if (ret != ERROR_OK)
		LOG_ERROR("Failed to write SPIFLASH_WRITE_ENABLE command : %d", ret);

	ret = inactivate_spi_ss(target);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to inactivate SPI SS : %d", ret);
		goto end;
	}

	if (!is_qspi_control_done(target)) {
		LOG_ERROR("Confirm SPI Control is Done failed");
		ret = ERROR_FAIL;
		goto end;
	}

	if (!is_write_enable(target, spi_ss))
		ret = ERROR_FAIL;

end:
	return ret;
}

static int write_mem_addr_to_flash(struct target *target, uint32_t mem_addr)
{
	int ret;
	uint8_t byte;

	// Write a 4 bytes address memmory to flash
	for (int i = 3; i >= 0; --i) {
		byte = (mem_addr >> (i * 8)) & 0xFF;
		ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR, byte);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to write byte %d of memory address: 0x%08x", i, mem_addr);
			break;
		}
	}

	return ret;
}

static int erase_sector(struct flash_bank *bank, unsigned int sector)
{
	int ret;
	struct target *target = bank->target;
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;
	uint32_t erase_cmd = scqspi_info->dev.erase_cmd;
	uint32_t mem_addr = scqspi_info->flash_addr + bank->sectors[sector].offset;

	ret = activate_spi_ss(target, scqspi_info->spi_ss);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to activate SPI SS : %d", ret);
		goto end;
	}

	LOG_DEBUG("Send Erase command : 0x%02x", erase_cmd);
	ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR, erase_cmd);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write Erase command : %d", ret);
		goto inactivate_ss;
	}

	LOG_DEBUG("Send Memory Address (4byte) : mem_addr : 0x%08x", mem_addr);
	ret = write_mem_addr_to_flash(target, mem_addr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write 4 bytes Memory Address : %d", ret);
		goto inactivate_ss;
	}

	if (!is_qspi_idle(target)) {
		ret = ERROR_FLASH_BUSY;
		goto end;
	}

inactivate_ss:
	ret = inactivate_spi_ss(target);
	if (ret != ERROR_OK)
		LOG_ERROR("Failed to inactivate SPI SS : %d", ret);

end:
	return ret;
}

static int qspi_erase_sector(struct flash_bank *bank, unsigned int sector)
{
	int ret;
	struct target *target = bank->target;
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;

	ret = set_write_enable(target, scqspi_info->spi_ss);
	if (ret != ERROR_OK)
		goto end;

	ret = erase_sector(bank, sector);
	if (ret != ERROR_OK)
		goto end;

	alive_sleep(SCQSPI_ERASE_BLOCK_WAIT_MS);

	if (!is_write_disable(target, scqspi_info->spi_ss))
		ret = ERROR_FAIL;

end:
	return ret;
}

static int write_data_to_flash(struct target *target, const uint8_t *data, uint32_t size)
{
	int ret;

	for (unsigned int i = 0; i < size; i++) {
		ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR, data[i]);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to write TX Data");
			break;
		}
	}
	return ret;
}

static int scqspi_memory_data_write(struct target *target, uint32_t spi_ss, uint32_t pprog_cmd,
	uint32_t mem_addr, uint32_t write_size,
	const uint8_t *write_data)
{
	int ret;

	ret = activate_spi_ss(target, spi_ss);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to activate SPI SS : %d", ret);
		goto end;
	}

	LOG_DEBUG("Send Page program instruction : 0x%08x", pprog_cmd);
	ret = target_write_u32(target, SCOBCA1_FPGA_NORFLASH_QSPI_TDR, pprog_cmd);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to send `Page program (4byte)` instruction");
		goto inactivate_ss;
	}

	LOG_DEBUG("Send Memory Address (4byte)");
	ret = write_mem_addr_to_flash(target, mem_addr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write memory address to flash");
		goto inactivate_ss;
	}

	if (!is_qspi_idle(target)) {
		ret = ERROR_FLASH_BUSY;
		goto inactivate_ss;
	}

	ret = write_data_to_flash(target, write_data, write_size);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write data to flash : %d", ret);
		goto inactivate_ss;
	}

	if (!is_qspi_idle(target)) {
		ret = ERROR_FLASH_BUSY;
		goto end;
	}

inactivate_ss:
	ret = inactivate_spi_ss(target);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to inactivate SPI SS : %d", ret);
		goto end;
	}

	if (!is_qspi_control_done(target)) {
		LOG_ERROR("Confirm SPI Control is Done failed");
		ret = ERROR_FAIL;
	}

end:
	return ret;
}

static int write_data(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
	uint32_t count)
{
	int ret;
	struct target *target = bank->target;
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;
	uint32_t pprog_cmd = scqspi_info->dev.pprog_cmd;
	uint32_t remaining;
	uint32_t mem_addr;
	uint32_t chunk_size;
	uint32_t bufpos = 0;

	if (scqspi_info->dev.pprog_cmd == 0x00) {
		LOG_ERROR("Program page not available for this device");
		ret = ERROR_FLASH_OPER_UNSUPPORTED;
		goto end;
	}

	remaining = count;
	mem_addr = scqspi_info->flash_addr + offset;

	while (remaining > 0) {
		chunk_size =
			remaining > SCQSPI_PAGE_BUFFER_BYTE ? SCQSPI_PAGE_BUFFER_BYTE : remaining;

		LOG_DEBUG("Write %d byte [0x%02x] to 0x%08x from bufpos %d (remaing %d byte)",
			chunk_size, buffer[bufpos], mem_addr, bufpos, remaining);

		LOG_DEBUG("Set to `Write Enable'");
		ret = set_write_enable(target, scqspi_info->spi_ss);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to set write enable : %d", ret);
			break;
		}

		LOG_DEBUG("Write Data");
		ret = scqspi_memory_data_write(target, scqspi_info->spi_ss, pprog_cmd, mem_addr,
				chunk_size, (buffer + bufpos));
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to write data to flash at address 0x%08x : %d", mem_addr,
				ret);
			break;
		}

		offset += SCQSPI_PAGE_BUFFER_BYTE;

		if (!is_write_disable(target, scqspi_info->spi_ss)) {
			ret = ERROR_FAIL;
			break;
		}

		mem_addr += chunk_size;
		bufpos += chunk_size;
		remaining -= chunk_size;
	}

end:
	return ret;
}

/* -------------------------------------------------------------------------
 * Command handler functions
 * -------------------------------------------------------------------------*/

/*
 * @brief Flash bank command handler for SC QSPI NOR devices.
 *
 * Syntax:
 *   flash bank scqspi <base> <size> <chip_width> <bus_width> <target#> <flash
 * start addr> <mem_no>
 *
 * <base> : Base address of the flash
 * <size> : Auto (0)
 * <chip_width> : Auto (0)
 * <bus_width> : Auto (0)
 * <target#> : Target to flash
 * <flash start addr> : Flash address where to start writing
 * <mem_no> : Configuration flash memory number
 */
FLASH_BANK_COMMAND_HANDLER(scqspi_flash_bank_command)
{
	struct scqspi_flash_bank *scqspi_info;

	if (CMD_ARGC < 8)
		return ERROR_COMMAND_SYNTAX_ERROR;

	scqspi_info = malloc(sizeof(struct scqspi_flash_bank));
	if (!scqspi_info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = scqspi_info;
	scqspi_info->probed = false;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], scqspi_info->flash_addr);
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[7], scqspi_info->mem_no);
	// For Configuration NOR flash, always 0x01
	scqspi_info->spi_ss = 0x01;

	return ERROR_OK;
}

static int scqspi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	int ret;
	struct target *target = bank->target;
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;

	LOG_DEBUG("%s: from sector %u to sector %u (base: 0x%08x)", __func__, first, last,
		scqspi_info->flash_addr);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(scqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (scqspi_info->dev.erase_cmd == 0x00) {
		LOG_ERROR("Sector erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (last < first || last >= bank->num_sectors) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			ret = ERROR_FLASH_PROTECTED;
			goto end;
		}
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		ret = qspi_erase_sector(bank, sector);
		if (ret != ERROR_OK) {
			LOG_ERROR("Flash sector_erase failed on sector %u : %d", sector, ret);
			break;
		}
		LOG_INFO("Flash sector %u offset %d", sector, bank->sectors[sector].offset);
		alive_sleep(SCQSPI_ERASE_SECTOR_WAIT_MS);
	}

end:
	return ret;
}

static int scqspi_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
	uint32_t count)
{
	int ret;
	struct target *target = bank->target;
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;

	LOG_DEBUG("%s: offset=0x%08x count=0x%08x (base: 0x%08x)", __func__, offset, count,
		scqspi_info->flash_addr);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		ret = ERROR_TARGET_NOT_HALTED;
		goto end;
	}

	if (!(scqspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		ret = ERROR_FLASH_BANK_NOT_PROBED;
		goto end;
	}

	if (offset + count > bank->size) {
		LOG_ERROR("Flash access out of range: offset=%u, count=%u, bank_size=%u", offset,
			count, bank->size);
		return ERROR_FLASH_DST_OUT_OF_BANK;
	}

	// Check sector protection
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		struct flash_sector *bs = &bank->sectors[sector];

		if ((offset < (bs->offset + bs->size)) && ((offset + count - 1) >= bs->offset) &&
			bs->is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			ret = ERROR_FAIL;
			goto end;
		}
	}

	ret = write_data(bank, buffer, offset, count);
	if (ret != ERROR_OK)
		LOG_ERROR("Failed to write data to flash : %d", ret);

end:
	return ret;
}

static int scqspi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int scqspi_probe(struct flash_bank *bank)
{
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0;

	int ret;

	if (scqspi_info->probed)
		free(bank->sectors);

	scqspi_info->probed = false;

	ret = scqspi_init(bank);
	if (ret != ERROR_OK)
		goto end;

	ret = scqspi_read_id(bank, &id);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read flash ID");
		goto end;
	}

	memset(&scqspi_info->dev, 0, sizeof(scqspi_info->dev));
	bool found = false;
	for (const struct flash_device *p = flash_devices; p->name; p++) {
		if (p->device_id == id) {
			memcpy(&scqspi_info->dev, p, sizeof(scqspi_info->dev));
			found = true;
			break;
		}
	}

	if (!found) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		ret = ERROR_FAIL;
		goto end;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")", scqspi_info->dev.name,
		scqspi_info->dev.device_id);

	bank->size = scqspi_info->dev.size_in_bytes;
	LOG_DEBUG("bank size %d", scqspi_info->dev.size_in_bytes);

	if (scqspi_info->dev.sectorsize == 0) {
		LOG_ERROR("No sectors found for this device");
		ret = ERROR_FAIL;
		goto end;
	}
	if (scqspi_info->dev.pagesize == 0) {
		LOG_ERROR("No page size found for this device");
		ret = ERROR_FAIL;
		goto end;
	}

	// Create and fill sectors array
	bank->num_sectors = scqspi_info->dev.size_in_bytes / scqspi_info->dev.sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		ret = ERROR_FAIL;
		goto end;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * scqspi_info->dev.sectorsize;
		sectors[sector].size = scqspi_info->dev.sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	scqspi_info->probed = true;

end:
	return ret;
}

static int scqspi_auto_probe(struct flash_bank *bank)
{
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;

	if (scqspi_info->probed)
		return ERROR_OK;

	return scqspi_probe(bank);
	;
}

static int scqspi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct scqspi_flash_bank *scqspi_info = bank->driver_priv;

	if (!(scqspi_info->probed)) {
		command_print_sameline(cmd, "QSPI flash bank not probed yet");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	command_print_sameline(cmd,
		"flash \'%s\', device id = 0x%06" PRIx32 ", flash size = %" PRIu32
		"%sB(page size = %" PRIu32 ", read = 0x%02" PRIx8 ", qread = 0x%02" PRIx8
		", pprog = 0x%02" PRIx8 ", mass_erase = 0x%02" PRIx8 ", sector size = %" PRIu32
		" %sB, sector_erase = 0x%02" PRIx8 ")",
		scqspi_info->dev.name,
		scqspi_info->dev.device_id,
		bank->size / 4096 ? bank->size / 1024 : bank->size,
		bank->size / 4096 ? "Ki" : "",
		scqspi_info->dev.pagesize,
		scqspi_info->dev.read_cmd,
		scqspi_info->dev.qread_cmd,
		scqspi_info->dev.pprog_cmd,
		scqspi_info->dev.chip_erase_cmd,
		scqspi_info->dev.sectorsize / 4096 ? scqspi_info->dev.sectorsize / 1024
						   : scqspi_info->dev.sectorsize,
		scqspi_info->dev.sectorsize / 4096 ? "Ki" : "",
		scqspi_info->dev.erase_cmd);

	return ERROR_OK;
}

static int scqspi_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;

	if (set) {
		LOG_WARNING("setting soft protection only, not related to flash's hardware "
			"write protection");
	}

	return ERROR_OK;
}

static int scqspi_verify(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
			 uint32_t count)
{
	/* Not implemented */
	return ERROR_OK;
}

const struct flash_driver scqspi_flash = {
	.name = "scqspi",
	.flash_bank_command = scqspi_flash_bank_command,
	.erase = scqspi_erase,
	.protect = scqspi_protect,
	.write = scqspi_write,
	.read = scqspi_read,
	.verify = scqspi_verify,
	.probe = scqspi_probe,
	.auto_probe = scqspi_auto_probe,
	.info = scqspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
