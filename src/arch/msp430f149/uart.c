/* uart.c
    
    Copyright (C) David Snowdon, Robert Reid, 2003. 
 
    Implementation of Scandal UART prototypes for MSP430. 
 
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

#include "scandal_types.h"
#include "scandal_uart.h"

void UART_SendByte(u08 Data){

}

void UART_flush_tx(void){
}	

u08 UART_ReceiveByte(void){
}

u08 UART_CheckReceived(void){
}

void buffer_received_char(u08 Data){
}

u08 UART_is_received(void){
}

void UART_PrintfU4(u08 Data){
}

void UART_Printfu08(u08 Data){
}

void UART_Printfu16(u16 Data){
}

void UART_Init(void){
}

void UART_baud_rate      (u32 rate, u32 clock_speed){
}
