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

int freertos_started = 0, to_feed_iwdg = 1, uptime = 0;

typedef int (*command_handler_t)(int argc, char **argv);
typedef struct {
	const char *name;
	command_handler_t handler;
	const char *help;
} command_t;

int help_cmd(int argc, char **argv);
int uptime_cmd(int argc, char **argv);

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

/* This is a ring buffer to holding characters as they are typed
 * it maintains both the place to put the next character received
 * from the UART, and the place where the last character was
 * read by the program. See the README file for a discussion of
 * the failure semantics.
 */

#define RECV_BUF_SIZE	128		/* Arbitrary buffer size */
char usart_rx_buf[RECV_BUF_SIZE];
volatile int recv_ndx_nxt;		/* Next place to store */
volatile int recv_ndx_cur;		/* Next place to read */

/* For interrupt handling we add a new function which is called
 * when recieve interrupts happen. The name (usart3_isr) is created
 * by the irq.json file in libopencm3 calling this interrupt for
 * USART3 'usart3', adding the suffix '_isr', and then weakly binding
 * it to the 'do nothing' interrupt function in vec.c.
 *
 * By defining it in this file the linker will override that weak
 * binding and instead bind it here, but you have to get the name
 * right or it won't work. And you'll wonder where your interrupts
 * are going.
 */
void
usart2_isr(void)
{
	uint32_t r;
	int i;

	do {
		r = USART_ISR(USART_CONSOLE);
		if (r & USART_ISR_RXNE) {
			usart_rx_buf[recv_ndx_nxt] = USART_RDR(USART_CONSOLE) & USART_RDR_MASK;

			/* Check for "overrun" */
			i = (recv_ndx_nxt + 1) % RECV_BUF_SIZE;
			if (i != recv_ndx_cur) {
				recv_ndx_nxt = i;
			}
		}
	} while ((r & USART_ISR_RXNE) != 0); /* can read back-to-back interrupts */
}

/*
 * char = console_getc(int wait)
 *
 * Check the console for a character. If the wait flag is
 * non-zero. Continue checking until a character is received
 * otherwise return 0 if called and no character was available.
 *
 * The implementation is a bit different however, now it looks
 * in the ring buffer to see if a character has arrived.
 */
char
console_getc(int wait)
{
	char c = 0;

	while ((wait != 0) && (recv_ndx_cur == recv_ndx_nxt)) {
		if (freertos_started == 1) taskYIELD();
	}

	if (recv_ndx_cur != recv_ndx_nxt) {
		c = usart_rx_buf[recv_ndx_cur];
		recv_ndx_cur = (recv_ndx_cur + 1) % RECV_BUF_SIZE;
	}
	return c;
}

/*
 * int console_gets(char *s, int len)
 *
 * Wait for a string to be entered on the console, limited
 * support for editing characters (back space and delete)
 * end when a <CR> character is received.
 */
int
console_gets(char *s, int len)
{
	char *t = s, c;

	if (s == NULL || len <= 0)
		return 0;

	*t = '\000';
	/* read until a <CR> is received */
	while ((c = console_getc(1)) != '\r') {
		if ((c == '\010') || (c == 0x7f)) {
			/* 0x8 = backspace, 0x7f = delete */
			if (t > s) {
				/* send ^H ^H to erase previous character */
				console_puts("\010 \010");
				t --;
			}
		} else {
			*t = c;
			console_putc(c);
			if ((t - s) < len) {
				t ++;
			}
		}
		/* update end of string with NUL */
		*t = '\000';
	}
	_putchar('\n'); /* print last \n' chars */
	return (t - s);
}

int
reset_cmd(int argc, char **argv)
{
	to_feed_iwdg = 0;
	scb_reset_system();
	return 0;
}

int
uptime_cmd(int argc, char **argv)
{
	uptime = xTaskGetTickCount() / (pdMS_TO_TICKS(1000));

	/* STM32WLxx has no FPU */
	mini_printf("rev %s built at %s %s, up %d secs, FreeRTOS %s\n", CONFIG_REVISION, __DATE__, __TIME__, uptime, tskKERNEL_VERSION_NUMBER);
	return 0;
}

const command_t main_cmds[] = {
	{
		.name = "reset",
		.handler = reset_cmd,
		.help = "soft reset system",
	},
	{
		.name = "uptime",
		.handler = uptime_cmd,
		.help = "print current uptime of system",
	},
	{
		.name = "help",
		.handler = help_cmd,
		.help = "this cmd",
	},
};

#define CMDS_SIZE (sizeof(main_cmds) / sizeof(main_cmds[0]))
#define MAX_ARGS 10

/* return 0 if cmds runs OK */
int
help_cmd(int argc, char **argv)
{
	int i;
	if (argc == 1)
		console_puts("avail cmds: ");
	for (i = 0; i < CMDS_SIZE; i ++) {
		if (argc == 2) {
			if (strcasecmp(argv[1], main_cmds[i].name) == 0) {
				/* cmd matched */
				console_puts(main_cmds[i].help);
				break;
			}
		} else {
			console_puts(main_cmds[i].name);
			if (i < (CMDS_SIZE - 1))
				console_putc(' ');
		}
	}
	_putchar('\n');
	return 0;
}

static void
cmdline_handler(void *unused)
{
	char gosh_prompt[] = "LORA> ";
	char cmd[RECV_BUF_SIZE], *argv[MAX_ARGS], *p;
	int len, i, argc, pos;

	while (1) {
		console_puts(gosh_prompt);
		len = console_gets(cmd, RECV_BUF_SIZE);
		if (len == 0)
			continue;
		argc = pos = 0;
		/* to find count of arguments */
		while (pos < len && argc < MAX_ARGS) {
			/* strip prefix ' ' & '\t' */
			while (pos < len && ((cmd[pos] == ' ') || (cmd[pos] == '\t')))
				pos ++;

			if (pos == len || cmd[pos] == '\0')
				break;
			p = cmd + pos;
			argv[argc ++] = p;

			while (pos < len && ((cmd[pos] != ' ') && (cmd[pos] != '\t')))
				pos ++;

			if (pos == len) break;
			else cmd[pos ++] = '\0';
		}

		if (argc > 0) {
			for (i = 0; i < CMDS_SIZE; i ++) {
				if (strcmp(argv[0], main_cmds[i].name) == 0) {
					if (main_cmds[i].handler(argc, argv)) {
						/* cmd runs failed, print help text */
						console_puts(main_cmds[i].help);
						_putchar('\n');
					}
					break;
				}
			}

			if (i == CMDS_SIZE)
				mini_printf("unknow cmd: %s, argc = %d\n", argv[0], argc);
		}
	}
}

static void
server_task(void *unused)
{
	TickType_t tick;

	while (1) {
		tick = xTaskGetTickCount();
		if (to_feed_iwdg)
			iwdg_reset(); /* reset IWDG */
		vTaskDelayUntil(&tick, pdMS_TO_TICKS(1000 - (tick % 1000))); /* loop every 1000ms */
	}
}

static void
init_task(void *unused)
{
	freertos_started = 1;
	mini_printf("INIT -> OK\n");

	/* start command line task */
	if (pdPASS != xTaskCreate(cmdline_handler, "CMD", 400, NULL, 2, NULL)) {
		console_puts("can't create command line task\n");
	}

	/* start main task */
	if (pdPASS != xTaskCreate(server_task, "GOD", 800, NULL, 4, NULL)) {
		console_puts("can't create server task\n");
	}

	vTaskDelay(pdMS_TO_TICKS(1000));

	/* delete itself now */
	vTaskDelete(NULL);
}

int
main(void)
{
	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

	rcc_clock_setup_pll(&(rcc_hsi16_configs[RCC_CLOCK_VRANGE1_36MHZ]));

	/* enable IWDG */
	rcc_osc_on(RCC_LSI); /* IWDG NEEDS LSI */
	iwdg_set_period_ms(3000); /* 3s */
	/* START IWDG */
	iwdg_start();

	usart2_setup();

	mini_printf("HSI16 %dMHZ -> OK\n", rcc_ahb_frequency / 1000000);

	if (pdPASS != xTaskCreate(init_task, "INIT", 500, NULL, 4, NULL)) {
		mini_printf("init task error\n");
	}

	iwdg_reset();
	/* reset IWDG before enter FreeRTOS */

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
