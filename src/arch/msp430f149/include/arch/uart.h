#ifndef __UART_H 
#define __UART_H

#include <scandal/types.h>

void UART_Init(uint32_t baudrate);
void UART_tx(char *buf, int len);
void UART_putchar(char c);

#endif /* end __UART_H */
