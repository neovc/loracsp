#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/iwdg.h>

#include "conf_loracsp.h"
#include "common.h"

const struct rcc_clock_scale rcc_hsi_16mhz_config =
	{ /* 48MHz PLL from HSI16 VR1 */
		.pllm = 4,
		.plln = 24,
		.pllp = RCC_PLLCFGR_PLLP(6),
		.pllq = RCC_PLLCFGR_PLLQ_DIV6,
		.pllr = RCC_PLLCFGR_PLLR_DIV2,
		.pll_source = RCC_PLLCFGR_PLLSRC_HSI16,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_NODIV,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
		FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 48000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 48000000,
	};

const struct rcc_clock_scale rcc_hse_32mhz_config =
	{ /* 48MHz PLL from HSE 32M */
		.pllm = 4,
		.plln = 12,
		.pllp = RCC_PLLCFGR_PLLP(6),
		.pllq = RCC_PLLCFGR_PLLQ_DIV2,
		.pllr = RCC_PLLCFGR_PLLR_DIV2,
		.pll_source = RCC_PLLCFGR_PLLSRC_HSE,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_NODIV,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
		FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 48000000,
		.apb1_frequency = 48000000,
		.apb2_frequency = 48000000,
	};

#if 0
static void
usart1_setup(void)
{
	/* Enable Clock for PBx */
	rcc_periph_clock_enable(RCC_GPIOB);
	/* Enable Clock for USART1 */
	rcc_periph_clock_enable(RCC_USART1);
	
	/* PB6 TX, PB7 RX, USART1 */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_set_af(GPIOB, GPIO_AF7, GPIO6 | GPIO7);
	
	/* 115200 8N1 */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	
	/* can't has high priority than MAX_SYSCALL_INTERRUPT_PRIORITY */
	nvic_set_priority(NVIC_USART1_IRQ, IRQ2NVIC_PRIOR(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1));
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Enable USART Receive interrupt. */
//	usart_enable_rx_interrupt(USART1);
	
	usart_enable(USART1);
}
#endif

#define USART_CONSOLE USART2

static void
usart2_setup(void)
{
	/* Enable Clock for PAx */
	rcc_periph_clock_enable(RCC_GPIOA);
	/* Enable Clock for USART2 */
	rcc_periph_clock_enable(RCC_USART2);
	
	/* PA2 TX, PA3 RX, USART2, AF7 */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
	
	/* 115200 8N1 */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	
	/* can't has high priority than MAX_SYSCALL_INTERRUPT_PRIORITY */
	nvic_set_priority(NVIC_USART2_IRQ, IRQ2NVIC_PRIOR(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1));
	nvic_enable_irq(NVIC_USART2_IRQ);
	
	/* Enable USART Receive interrupt. */
//	usart_enable_rx_interrupt(USART2);
	
	usart_enable(USART2);
}

void
console_putc(const char c)
{
	int i = 0;

	/* check usart's tx buffer is empty */
	while ((i < USART_LOOP) && ((USART_ISR(USART_CONSOLE) & USART_ISR_TXE) == 0))
		i ++;
	if (i < USART_LOOP)
		usart_send(USART_CONSOLE, c);
}

/*
 * void console_puts(char *s)
 *
 * Send a string to the console, one character at a time, return
 * after the last character, as indicated by a NUL character, is
 * reached.
 */
void
console_puts(const char *s)
{
	int i = 0;
	
	if (s == NULL) return;
	
	while (s[i] != '\000') {
		console_putc(s[i]);
		/* Add in a carraige return, after sending line feed */
		if (s[i] == '\n')
			console_putc('\r');
		i ++;
	}
}

/* for tiny printf.c */
void
_putchar(const char c)
{
	console_putc(c);
	if (c == '\n')
		console_putc('\r');
}

static void
init_task(void *unused)
{
	mini_printf("INIT -> OK\n");
	vTaskDelay(1000);
	vTaskDelete(NULL);
}

void
clock_setup_pll(const struct rcc_clock_scale *clock)
{
	/* Enable internal high-speed oscillator (HSI16). */
	rcc_osc_on(RCC_HSI16);
	rcc_wait_for_osc_ready(RCC_HSI16);

	/* Select HSI16 as SYSCLK source. */
	rcc_set_sysclk_source(RCC_PLLCFGR_PLLSRC_HSI16);

	/* Enable external high-speed oscillator (HSE). */
	if (clock->pll_source == RCC_PLLCFGR_PLLSRC_HSE) {
		rcc_osc_on(RCC_HSE);
		rcc_wait_for_osc_ready(RCC_HSE);
	}

	pwr_set_vos_scale(clock->voltage_scale);
	pwr_disable_backup_domain_write_protect();

	/*
	* Set prescalers for AHB, ADC, APB1, APB2.
	* Do this before touching the PLL (TODO: why?).
	*/
	rcc_set_hpre(clock->hpre);
	rcc_set_ppre1(clock->ppre1);
	rcc_set_ppre2(clock->ppre2);

	/* Disable PLL oscillator before changing its configuration. */
	rcc_osc_off(RCC_PLL);

	/* Configure the PLL oscillator. */
	rcc_set_main_pll(clock->pll_source, clock->pllm, clock->plln,
			clock->pllp, clock->pllq,  clock->pllr);

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Configure flash settings. */
	if (clock->flash_config & FLASH_ACR_DCEN) {
		flash_dcache_enable();
	} else {
		flash_dcache_disable();
	}
	if (clock->flash_config & FLASH_ACR_ICEN) {
		flash_icache_enable();
	} else {
		flash_icache_disable();
	}

	flash_set_ws(clock->flash_config);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL);

	/* Wait for PLL clock to be selected. */
#if 0
	while ((i < 1000000) && (((RCC_CFGR >> RCC_CFGR_SWS_SHIFT) & RCC_CFGR_SWS_MASK)
		!= RCC_CFGR_SWS_PLL)) i ++;
#endif
	rcc_wait_for_sysclk_status(RCC_PLL);

	/* Set the peripheral clock frequencies used. */
	rcc_ahb_frequency  = clock->ahb_frequency;
	rcc_apb1_frequency = clock->apb1_frequency;
	rcc_apb2_frequency = clock->apb2_frequency;

	/* Disable internal high-speed oscillator. */
	if (clock->pll_source == RCC_PLLCFGR_PLLSRC_HSE) {
		rcc_osc_off(RCC_HSI16);
	}
}

int
main(void)
{
	clock_setup_pll(&(rcc_hsi_16mhz_config));

	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

	usart2_setup();

	mini_printf("START OK\n");

	if (pdPASS != xTaskCreate(init_task, "INIT", 500, NULL, 4, NULL)) {
		mini_printf("init task error\n");
	}

	vTaskStartScheduler();

	return 0;
}

/* based on code from book 'Beginning STM32' */
extern void vPortSVCHandler( void ) __attribute__ (( naked ));
extern void xPortPendSVHandler( void ) __attribute__ (( naked ));
extern void xPortSysTickHandler( void );

void sv_call_handler(void) {
	vPortSVCHandler();
}

void pend_sv_handler(void) {
	xPortPendSVHandler();
}

void sys_tick_handler(void) {
	xPortSysTickHandler();
}
