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
//<<<<<<< scandal_uart.h
extern void UART_Init           (void); 
//=======
//extern int UART_Init           (void); 
//>>>>>>> 1.15
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


/* Utilities */
void print_hex(u08 byte);
void print_string(u08*	buf);
u08 read_signed_num(s32* num);
void print_int(s32 val);


#endif 
