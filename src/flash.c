#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <libopencm3/stm32/flash.h>

#include "common.h"

#define FLASH_LOOP	1000000
#define FLASH_PAGESIZE	2048

uint32_t device_flash_size;
uint8_t flash_memory_ptr[FLASH_PAGESIZE];
uint32_t flash_eccc_count = 0, flash_eccd_count = 0;

extern void print_hex(uint8_t *src, int len);

int flash_read_data(uint8_t *data, uint32_t start, int length);
int flash_write_data(uint8_t *data, uint32_t start, int length);

#define MIDDLE_PAGE_NUM (device_flash_size / FLASH_PAGESIZE / 2)

/* return 0 if OK
 * return 1 if timeout
 */
int
flash_timeout_wait_for_last_operation(void)
{
	int i = 0;
	while ((i < FLASH_LOOP) && ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY))
		i ++;

	return i < FLASH_LOOP?0:1;
}

/* return actual byte written to flash */
int
flash_write_double_word(uint32_t address, uint32_t u1, uint32_t u2)
{
	/* Ensure that all flash operations are complete. */
	if (flash_timeout_wait_for_last_operation())
		return 0;

	if (FLASH_SR & FLASH_SR_PESD) /* Program/erase operation suspended */
		return 0;

	/* clear error status first */
	flash_clear_status_flags();

	/* Program the double word. */
	/* according RM0461 v8, P78, 3.3.8
	 * 1) Write a first word in an address aligned with double word
	 * 2) Write the second word
	 *
	 * DON'T USE MMIO64(address) here
	 */
	MMIO32(address) = u1;
	MMIO32(address + 4) = u2;

	/* Wait for the write to complete. */
	flash_wait_for_last_operation();

	return 8;
}

/* return 0 if OK
 * return 1 if failed
 */
int
flash_reset_page(uint32_t page)
{
	/* Ensure that all flash operations are complete. */
	if (flash_timeout_wait_for_last_operation())
		return 1;

	if (FLASH_SR & FLASH_SR_PESD) /* Program/erase operation suspended */
		return 1;

	flash_clear_status_flags();

	/* clear BKER flag first
	 * we need to clear FLASH_CR_PG flag for page erase
	 */
	FLASH_CR &= ~((FLASH_CR_PNB_MASK << FLASH_CR_PNB_SHIFT) | FLASH_CR_FSTPG | FLASH_CR_PG);
	FLASH_CR |= (page << FLASH_CR_PNB_SHIFT) | FLASH_CR_PER;
	FLASH_CR |= FLASH_CR_START;

	flash_wait_for_last_operation();
	return 0;
}

/* return 0 if OK
 * return 1 if failed
 */
int
flash_reset_all_pages(void)
{
	if (flash_timeout_wait_for_last_operation())
		return 1;
	FLASH_CR &= ~FLASH_CR_PG;
	FLASH_CR |= FLASH_CR_MER1;
	FLASH_CR |= FLASH_CR_START;

	flash_wait_for_last_operation();
	return 0;
}

/* return actual bytes write to flash */
int
flash_write_data(uint8_t *data, uint32_t start, int len)
{
	int i, j, k = 0, size;
	uint32_t page_start;
	uint32_t u32_1, u32_2;

	/* check if start_address is in proper range */
	if ((start < (FLASH_BASEADDR)) || (start >= (FLASH_BASEADDR + device_flash_size)))
		return 0;

	/* Ensure that all flash operations are complete. */
	if (flash_timeout_wait_for_last_operation())
		return 0;

	flash_unlock();

	flash_clear_status_flags();

	if (start % FLASH_PAGESIZE == 0) {
		/* start of flash page size */
		page_start = start;
		size = FLASH_PAGESIZE;
	} else {
		page_start = (start / FLASH_PAGESIZE) * FLASH_PAGESIZE;
		size = page_start + FLASH_PAGESIZE - start;
	}

	for (i = 0; i < len; ) {
		if (FLASH_PAGESIZE != (j = flash_read_data(flash_memory_ptr, page_start, FLASH_PAGESIZE))) {
			break;
		}

		if ((len - i) < size)
			size = len - i;

		memcpy(flash_memory_ptr + start - page_start, data, size);

		if (flash_reset_page((page_start - FLASH_BASEADDR) / FLASH_PAGESIZE)) {
			break;
		}

		/*
		 * we need to disable dcache before programing flash, re-enable dcache after programming flash.
		 */

		/* Disable DCache */
		flash_dcache_disable();
		flash_clear_status_flags();

		/* Enable writes to flash. */
		FLASH_CR &= ~((FLASH_CR_PNB_MASK << FLASH_CR_PNB_SHIFT) | FLASH_CR_FSTPG | FLASH_CR_PER);
		FLASH_CR |= FLASH_CR_PG;

		for (j = 0; j < FLASH_PAGESIZE; j += sizeof(uint64_t), page_start += sizeof(uint64_t)) {
			u32_1 = *(uint32_t *)(flash_memory_ptr + j);
			u32_2 = *(uint32_t *)(flash_memory_ptr + j + sizeof(uint32_t));
			if (8 != flash_write_double_word(page_start, u32_1, u32_2))
				break;
		}

		/* Disable writes to flash. */
		FLASH_CR &= ~FLASH_CR_PG;

		/* Reset DCache */
		flash_dcache_reset();
		/* Enable DCache */
		flash_dcache_enable();

		if (j != FLASH_PAGESIZE) /* sth is wrong */
			break;

		i += size;
		data += size;
		k += size;
		size = FLASH_PAGESIZE;
	}

	flash_lock();
	return k;
}

/* return actual read byte from flash */
int
flash_read_data(uint8_t *data, uint32_t start, int length)
{
	int i;
	uint32_t p;
	uint32_t *m= (uint32_t *) start;

	if (length <= 0 || data == NULL || (start % 4)) /* start address should be 4 byte align */
		return 0;

	if (start < FLASH_BASEADDR || start >= (FLASH_BASEADDR + device_flash_size))
		return 0;

	p = (int) data;
	if (p % 4) {
		return 0;
	}

	if ((length % 4) == 0) {
		length = length / 4;
	} else {
		length = (length / 4 + 1);
	}

	for (i = 0; i < length; i ++) {
		*((uint32_t *) data) = *(m + i);
		data += 4;
	}

	return i * 4;
}

void flash_isr(void)
{
	if (FLASH_ECCR & FLASH_ECCR_ECCC) {
		FLASH_ECCR |= FLASH_ECCR_ECCC; /* CLEAR ECCC FLAG */
		flash_eccc_count ++;
	}
}

void
init_flash(void)
{
	if (FLASH_ECCR & FLASH_ECCR_ECCD) {
		console_puts("FLASH ECC ERROR DETECTED\n");
		FLASH_ECCR |= FLASH_ECCR_ECCD; /* CLEAR ECCD FLAG */
		flash_eccd_count ++;
	}

	if (FLASH_ECCR & FLASH_ECCR_ECCC) {
		console_puts("FLASH ECC CORRECTION DETECTED\n");
		FLASH_ECCR |= FLASH_ECCR_ECCC; /* CLEAR ECCC FLAG */
		flash_eccc_count ++;
	}

	FLASH_ECCR |= FLASH_ECCR_ECCIE; /* enable ECC interrupt */
}

int
flash_cmd(int argc, char **argv)
{
	int len, r;
	uint8_t data[64];
	uint32_t addr;

	if (argc > 3 && strcasecmp(argv[1], "read") == 0) {
		/* fram read addr len */
		addr = atoi(argv[2]);
		len = atoi(argv[3]);
		if (len > 64)
			len = 64;
		r = flash_read_data(data, FLASH_BASEADDR + addr, len);
		mini_printf("read len #%d from addr 0x%x return %d\n", len, addr, r);
		print_hex(data, r);
		return 0;
	} else if (argc > 3 && strcasecmp(argv[1], "write") == 0) {
		/* fram write addr data */
		addr = atoi(argv[2]);
		len = strlen(argv[3]);
		if (len > 64)
			len = 64;
		memcpy(data, argv[3], len);
		r = flash_write_data(data, FLASH_BASEADDR + addr, len);
		mini_printf("write data len #%d to addr 0x%x return %d\n", len, addr, r);
		return 0;
	}
	return 1;
}
