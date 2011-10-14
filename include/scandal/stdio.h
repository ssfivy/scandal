#ifndef _SCANDAL_STDIO_
#define _SCANDAL_STDIO_

int UART_printf(const char *format, ...) __attribute__ ((format (printf, 1, 2)));

#endif
