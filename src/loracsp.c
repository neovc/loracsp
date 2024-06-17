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
	usart_enable_rx_interrupt(USART2);
	
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

int
main(void)
{
	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

	rcc_clock_setup_pll(&(rcc_hsi16_configs[RCC_CLOCK_VRANGE1_36MHZ]));

	usart2_setup();

	mini_printf("HSI16 %dMHZ -> OK\n", rcc_ahb_frequency / 1000000);

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
