#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <stdarg.h>

#define IRQ2NVIC_PRIOR(x)  ((x)<<4)

#define mini_printf     printf_
#define USART_LOOP      10000

int printf_(const char *format, ...);
void console_puts(const char *); 
void console_putc(const char);

#endif
