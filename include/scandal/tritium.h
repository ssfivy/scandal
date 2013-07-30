/*
 * Tritium TRI63 CAN Driver Controls header
 * Copyright (c) 2006, Tritium Pty Ltd.  All rights reserved.
 *  
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *	- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
 *	  in the documentation and/or other materials provided with the distribution.
 *	- Neither the name of Tritium Pty Ltd nor the names of its contributors may be used to endorse or promote products 
 *	  derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * Last Modified: D. Snowdon, 31 June, 2008
 *
 */

#ifndef __TRITIUM_H__
#define __TRITIUM_H__

// Typedefs for quickly joining multiple bytes/ints/etc into larger values
// These rely on byte ordering in CPU & memory - i.e. they're not portable across architectures
typedef union _group_64 {
	float data_fp[2];
	unsigned char data_u8[8];
	unsigned int data_u16[4];
	unsigned long data_u32[2];
} group_64;

typedef union _group_32 {
	float data_fp;
	unsigned char data_u8[4];
	unsigned int data_u16[2];
	unsigned long data_u32;
} group_32;

typedef union _group_16 {
	unsigned char data_u8[2];
	unsigned int data_u16;
} group_16;

// Driver controls CAN base address and packet offsets
#define DC_BASE					0x0500
#define DC_DRIVE				DC_BASE+1
#define DC_POWER				DC_BASE+2
#define DC_RESET				DC_BASE+3
#define DC_SWITCH				DC_BASE+4

#define MC_BASE					0x400
#define MC_STATUS				MC_BASE+1
//#define MC_BUS				MC_BASE+2
//define MC_VELOCITY			MC_BASE+3
//#define MC_PHASE				MC_BASE+4
#define MC_MOTOR_VOLTAGE_VECTOR	MC_BASE+5
#define MC_MOTOR_CURRENT_VECTOR	MC_BASE+6
#define MC_MOTOR_BACKEMF_VECTOR	MC_BASE+7
#define MC_MOTOR_15V_1V65		MC_BASE+8
#define MC_MOTOR_2V5_1V2		MC_BASE+9
#define MC_FAN_SPEED			MC_BASE+10
#define MC_HEATSINK_MOTOR_TEMP	MC_BASE+11
#define MC_AIR_IN_CPU_TEMP		MC_BASE+12
#define MC_AIR_OUT_CAP_TEMP		MC_BASE+13
#define MC_ODOMETER_BUSAMPHOURS	MC_BASE+14

// Status values (for message reception)
#define CAN_ERROR		0xFFFF
#define CAN_MERROR		0xFFFE
#define CAN_WAKE		0xFFFD
#define CAN_RTR			0xFFFC
#define CAN_OK			0x0001

#endif
