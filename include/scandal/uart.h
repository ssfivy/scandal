/* 
 * scandal_uart.h
 * 
 * Created by Robert Reid, David Snowdon, 2003. 
 * Copyright (C) 2003 Robert Reid, David Snowdon. 
 * 
 * UART function prototypes (to be implemented by platform-specific code)
 * 
 */

/* 
 * This file is part of Scandal.
 * 
 * Scandal is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * Scandal is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with Scandal.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __UART_H__
#define __UART_H__

#include <scandal/types.h>

/* UART Default baud rate definition */ 
#define UART_DEFAULT_BAUD         115200         /* baud rate*/ 
 
/* Parameters */ 
extern void UART_baud_rate      (u32 rate, u32 clock_speed); 
/* Global functions */ 
extern void UART_SendByte       (u08 Data); 
extern void UART_flush_tx	(void);
extern u08  UART_ReceiveByte    (void);
extern u08  UART_ReceiveByte_to (u08* byte, u16 timeout); 
extern void UART_PrintfProgStr  (u08* pBuf); 
extern void UART_PrintfEndOfLine(void); 
extern void UART_Printfu08      (u08 Data); 
extern void UART_Printfu16      (u16 Data); 
extern void UART_Init           (uint32_t baudrate); 
extern u08  UART_CheckReceived	(void);
extern u08  UART_is_received	(void);
extern void UART_Flush          (void);
extern void init_buffer		(void);
extern int remaining_buffer	(void);
extern void insert_buffer	(uint8_t elem);
extern uint8_t remove_buffer	(void);

extern void DelayMS		(u16 time);
extern void UART_Break		(void);

extern void UART_SendBytePacket	(char * buf, int buf_len);
extern void UART_ClearReceive   (void);
/* Macros */ 
#define EOL           UART_PrintfEndOfLine 


/* New buffer managed UART stuff */

#define UART_BUFFER_OVERFLOW -1
#define UART_NO_LINE -2

struct UART_buffer_descriptor {
	char *buf;
	uint32_t size;
	uint32_t write_pos;
	uint8_t overflow;
};

char *UART_readline_double_buffer(struct UART_buffer_descriptor *desc_1, struct UART_buffer_descriptor *desc_2);
void UART_init_double_buffer(struct UART_buffer_descriptor *desc_1, char *buf_1, uint32_t size_1,
	struct UART_buffer_descriptor *desc_2, char *buf_2, uint32_t size_2);

/* Utilities */
void print_hex(u08 byte);
void print_string(u08*	buf);
u08 read_signed_num(s32* num);
void print_int(s32 val);


#endif 
