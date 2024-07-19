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

int freertos_started = 0, to_feed_iwdg = 1;
uint32_t uptime = 0;
uint8_t csp_node = 0, boot_cause = 0;
const char status[2][8] = {"OK", "FAILED"};

extern uint32_t device_flash_size;
extern void init_flash(void);
extern int save_params_config(void);
extern int read_params_config(void);

typedef int (*command_handler_t)(int argc, char **argv);
typedef struct {
	const char *name;
	command_handler_t handler;
	const char *help;
} command_t;

int help_cmd(int argc, char **argv);
int uptime_cmd(int argc, char **argv);
extern int param_cmd(int argc, char **argv);
extern int flash_cmd(int argc, char **argv);
extern int lora_cmd(int argc, char **argv);

uint8_t get_boot_cause(void);

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
 * when recieve interrupts happen. The name (usart2_isr) is created
 * by the irq.json file in libopencm3 calling this interrupt for
 * USART2 'usart2', adding the suffix '_isr', and then weakly binding
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

	mini_printf("rev %s built at %s %s, up %d secs (%.2f days), FreeRTOS %s\n", CONFIG_REVISION, __DATE__, __TIME__, uptime,
			1.0 * uptime / 86400.0, tskKERNEL_VERSION_NUMBER);
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
		.name = "flash",
		.handler = flash_cmd,
		.help = "flash [read|write] addr [len|data]",
	},
	{
		.name = "param",
		.handler = param_cmd,
		.help = "param [list|set|save] [name] [value]",
	},
	{
		.name = "lora",
		.handler = lora_cmd,
		.help = "lora [init|send] [data]",
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

	save_params_config();

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
	int r;

	device_flash_size = MMIO16(DESIG_FLASH_SIZE_BASE) * 1024;

	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

	rcc_clock_setup_pll(&(rcc_hsi16_configs[RCC_CLOCK_VRANGE1_36MHZ]));

	/* enable IWDG */
	rcc_osc_on(RCC_LSI); /* IWDG NEEDS LSI */
	rcc_wait_for_osc_ready(RCC_LSI);
	iwdg_set_period_ms(3000); /* 3s */
	/* START IWDG */
	iwdg_start();

	usart2_setup();

	FLASH_ACR &= ~FLASH_ACR_PES; /* CLEAR PES FLAG */
	r = read_params_config();

	boot_cause = get_boot_cause();

	mini_printf("READ PARAMS FROM FLASH -> %s\n", status[r]);
	mini_printf("CSP NODE -> #%d\n", csp_node);

	init_flash();

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

/* polynomial 0x1EDC6F41 */
static const uint32_t
crc_tab[256] =
{
	0x00000000, 0xF26B8303, 0xE13B70F7, 0x1350F3F4, 0xC79A971F, 0x35F1141C, 0x26A1E7E8, 0xD4CA64EB,
	0x8AD958CF, 0x78B2DBCC, 0x6BE22838, 0x9989AB3B, 0x4D43CFD0, 0xBF284CD3, 0xAC78BF27, 0x5E133C24,
	0x105EC76F, 0xE235446C, 0xF165B798, 0x030E349B, 0xD7C45070, 0x25AFD373, 0x36FF2087, 0xC494A384,
	0x9A879FA0, 0x68EC1CA3, 0x7BBCEF57, 0x89D76C54, 0x5D1D08BF, 0xAF768BBC, 0xBC267848, 0x4E4DFB4B,
	0x20BD8EDE, 0xD2D60DDD, 0xC186FE29, 0x33ED7D2A, 0xE72719C1, 0x154C9AC2, 0x061C6936, 0xF477EA35,
	0xAA64D611, 0x580F5512, 0x4B5FA6E6, 0xB93425E5, 0x6DFE410E, 0x9F95C20D, 0x8CC531F9, 0x7EAEB2FA,
	0x30E349B1, 0xC288CAB2, 0xD1D83946, 0x23B3BA45, 0xF779DEAE, 0x05125DAD, 0x1642AE59, 0xE4292D5A,
	0xBA3A117E, 0x4851927D, 0x5B016189, 0xA96AE28A, 0x7DA08661, 0x8FCB0562, 0x9C9BF696, 0x6EF07595,
	0x417B1DBC, 0xB3109EBF, 0xA0406D4B, 0x522BEE48, 0x86E18AA3, 0x748A09A0, 0x67DAFA54, 0x95B17957,
	0xCBA24573, 0x39C9C670, 0x2A993584, 0xD8F2B687, 0x0C38D26C, 0xFE53516F, 0xED03A29B, 0x1F682198,
	0x5125DAD3, 0xA34E59D0, 0xB01EAA24, 0x42752927, 0x96BF4DCC, 0x64D4CECF, 0x77843D3B, 0x85EFBE38,
	0xDBFC821C, 0x2997011F, 0x3AC7F2EB, 0xC8AC71E8, 0x1C661503, 0xEE0D9600, 0xFD5D65F4, 0x0F36E6F7,
	0x61C69362, 0x93AD1061, 0x80FDE395, 0x72966096, 0xA65C047D, 0x5437877E, 0x4767748A, 0xB50CF789,
	0xEB1FCBAD, 0x197448AE, 0x0A24BB5A, 0xF84F3859, 0x2C855CB2, 0xDEEEDFB1, 0xCDBE2C45, 0x3FD5AF46,
	0x7198540D, 0x83F3D70E, 0x90A324FA, 0x62C8A7F9, 0xB602C312, 0x44694011, 0x5739B3E5, 0xA55230E6,
	0xFB410CC2, 0x092A8FC1, 0x1A7A7C35, 0xE811FF36, 0x3CDB9BDD, 0xCEB018DE, 0xDDE0EB2A, 0x2F8B6829,
	0x82F63B78, 0x709DB87B, 0x63CD4B8F, 0x91A6C88C, 0x456CAC67, 0xB7072F64, 0xA457DC90, 0x563C5F93,
	0x082F63B7, 0xFA44E0B4, 0xE9141340, 0x1B7F9043, 0xCFB5F4A8, 0x3DDE77AB, 0x2E8E845F, 0xDCE5075C,
	0x92A8FC17, 0x60C37F14, 0x73938CE0, 0x81F80FE3, 0x55326B08, 0xA759E80B, 0xB4091BFF, 0x466298FC,
	0x1871A4D8, 0xEA1A27DB, 0xF94AD42F, 0x0B21572C, 0xDFEB33C7, 0x2D80B0C4, 0x3ED04330, 0xCCBBC033,
	0xA24BB5A6, 0x502036A5, 0x4370C551, 0xB11B4652, 0x65D122B9, 0x97BAA1BA, 0x84EA524E, 0x7681D14D,
	0x2892ED69, 0xDAF96E6A, 0xC9A99D9E, 0x3BC21E9D, 0xEF087A76, 0x1D63F975, 0x0E330A81, 0xFC588982,
	0xB21572C9, 0x407EF1CA, 0x532E023E, 0xA145813D, 0x758FE5D6, 0x87E466D5, 0x94B49521, 0x66DF1622,
	0x38CC2A06, 0xCAA7A905, 0xD9F75AF1, 0x2B9CD9F2, 0xFF56BD19, 0x0D3D3E1A, 0x1E6DCDEE, 0xEC064EED,
	0xC38D26C4, 0x31E6A5C7, 0x22B65633, 0xD0DDD530, 0x0417B1DB, 0xF67C32D8, 0xE52CC12C, 0x1747422F,
	0x49547E0B, 0xBB3FFD08, 0xA86F0EFC, 0x5A048DFF, 0x8ECEE914, 0x7CA56A17, 0x6FF599E3, 0x9D9E1AE0,
	0xD3D3E1AB, 0x21B862A8, 0x32E8915C, 0xC083125F, 0x144976B4, 0xE622F5B7, 0xF5720643, 0x07198540,
	0x590AB964, 0xAB613A67, 0xB831C993, 0x4A5A4A90, 0x9E902E7B, 0x6CFBAD78, 0x7FAB5E8C, 0x8DC0DD8F,
	0xE330A81A, 0x115B2B19, 0x020BD8ED, 0xF0605BEE, 0x24AA3F05, 0xD6C1BC06, 0xC5914FF2, 0x37FACCF1,
	0x69E9F0D5, 0x9B8273D6, 0x88D28022, 0x7AB90321, 0xAE7367CA, 0x5C18E4C9, 0x4F48173D, 0xBD23943E,
	0xF36E6F75, 0x0105EC76, 0x12551F82, 0xE03E9C81, 0x34F4F86A, 0xC69F7B69, 0xD5CF889D, 0x27A40B9E,
	0x79B737BA, 0x8BDCB4B9, 0x988C474D, 0x6AE7C44E, 0xBE2DA0A5, 0x4C4623A6, 0x5F16D052, 0xAD7D5351
};

uint32_t
crc32_memory(uint8_t * data, uint32_t length)
{
	uint32_t crc = 0xFFFFFFFF;

	if (data == NULL || length == 0)
		return crc;

	while (length --)
		crc = crc_tab[(crc ^ *data++) & 0xFFL] ^ (crc >> 8);
	return (crc ^ 0xFFFFFFFF);
}

uint8_t
get_boot_cause(void)
{
	uint8_t r;

	r = (RCC_CSR >> 24) & 0xff;

	RCC_CSR |= RCC_CSR_RMVF; /* clear reset flag */

	if (r & 0x20) {
		/* iwdg reset */
		return 1;
	} else if (r & 0x10) {
		/* software reset */
		return 2;
	} else if (r & 0x40) {
		/* window watchdog reset */
		return 3;
	} else if (r & 0x80) {
		/* lower power reset */
		return 4;
	} else if (r & 0x01) {
		/* firewall reset */
		return 5;
	} else if (r & 0x02) {
		/* option byte loader reset */
		return 6;
	} else if (r & 0x0C) {
		/* power off/on, hardware watchdog */
		return 7;
	} else {
		/* other reset */
		return 0xff;
	}
}
