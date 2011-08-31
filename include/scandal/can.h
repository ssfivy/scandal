/*
 *  scandal_can.h
 *
 *  Created by David Snowdon on Wed Jul 24, 2002.
 *  Copyright (C) David Snowdon 2002. 
 *
 *  Defines a basic interface to a CAN controller.
 *  It is possible that this layer should be replaced with CANpie at some point, but adapting higher
 *  layers to such a standardised interface should not be difficult.
 *
 *  Used by scandal_engine to interface with CAN hardware/network.
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

#ifndef __SCANDAL_CAN__
#define __SCANDAL_CAN__

/* Includes */
#include <scandal/types.h>

/* Constant definitions */

/* Baud Rate Definitions */
#define SCANDAL_B1000    0
#define SCANDAL_B500     1
#define SCANDAL_B250     2
#define SCANDAL_B125     3
#define SCANDAL_B50      4
#define SCANDAL_B10      5
#define SCANDAL_NUM_BAUD 6 /* Number of baud rates */

#define MAX_BAUD         SCANDAL_B1000
#define MIN_BAUD         SCANDAL_B10
#define DEFAULT_BAUD     SCANDAL_B50

/* CAN Properties */
#define CAN_MSG_MAXSIZE  8

/* Types */
typedef struct can_mg {
  u32 id;
  u08 data[CAN_MSG_MAXSIZE];
  u08 length;
} can_msg;

/* Standard CAN Layer Prototypes */
/*! Initialise the controller such that it is scandal compliant,
    using the correct baud rate (DEFAULT_BAUD) */
void init_can(void);

/*! Get a message from the CAN controller. */
u08  can_get_msg(can_msg* msg);

/*! Send a message using the CAN controller */
u08  can_send_msg(can_msg* msg, u08 priority);

/*! Register a message ID/mask. This guarantees that these messages will
  not be filtered out by hardware filters. Other messages are not
  guaranteed */
u08  can_register_id(u32 mask, u32 data, u08 priority);

/*! Should be called when the CAN controller has an interrupt */
/*! \todo This is probably not the right location for this */
void can_interrupt(void);

/*! Should be called when there is idle time available and the CAN
	controller is able to do some housekeeping */
void can_poll(void);

/* Parameter settings */
u08  can_baud_rate(u08 mode);

/* Enable and disable CAN interrupts */
void  enable_can_interrupt(void);
void  disable_can_interrupt(void);

#endif
