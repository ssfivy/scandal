/*
 *  scandal_engine.h
 *
 *  Created by David Snowdon on Wed, 31 Jul, 2002.
 *  Copyright (c) David Snowdon 2009.
 *
 *  Core of the scandal engine with HLP.
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


#ifndef __SCANDAL_ENGINE__
#define __SCANDAL_ENGINE__

#include <scandal/types.h>
#include <scandal/can.h>
#include <scandal/timer.h>
#include <scandal/obligations.h>

/* Types */
/* Channel representative structure */
typedef struct in_chan{
	s32	value;
	u32 time;
	u32	rcvd_time;
} in_channel;

//#warning SIZEOF_IN_CHAN_CONFIG MAY ONLY WORK FOR MSP430
#define SIZEOF_IN_CHAN_CONFIG   (1 + 2 + 1) /* 3 bytes - does this get padded by the compiler? YES */ 
typedef struct in_chan_config{
	u08	source_node;
	u16	source_num;
}in_channel_config;

#define SIZEOF_OUT_CHAN_CONFIG  (4 + 4)
typedef struct out_chan_config{
	s32	m;
	s32	b;
}out_channel_config;

/* Global configuration */
#define HEARTBEAT_PERIOD		1000
#define SCANDAL_VERSION			0x0A		/* Version 0.10 */

/* Message type definitions */
#define CHANNEL_TYPE  	                0
#define CONFIG_TYPE			1
#define HEARTBEAT_TYPE 	                2
#define SCANDAL_ERROR_TYPE		3
#define USER_ERROR_TYPE                 4
#define RESET_TYPE			5                
#define USER_CONFIG_TYPE		6 
#define COMMAND_TYPE			7
#define TIMESYNC_TYPE                   8

/* Frame Definition #defines */

/* Common to all frames */
#define PRI_BITS		        3
#define PRI_OFFSET		        26

#define TYPE_BITS		        8
#define TYPE_OFFSET		        18

/* Channels */
#define CHANNEL_SOURCE_ADDR_BITS	8
#define CHANNEL_SOURCE_ADDR_OFFSET	10

#define CHANNEL_NUM_BITS	        10
#define CHANNEL_NUM_OFFSET  	        0

/* Configuration messages (both user and scandal) */
#define CONFIG_NODE_ADDR_BITS	        8
#define CONFIG_NODE_ADDR_OFFSET	        10

#define CONFIG_PARAM_BITS	        8
#define CONFIG_PARAM_OFFSET	        0

/* Heartbeat messages */
#define HEARTBEAT_NODE_ADDR_BITS	8
#define HEARTBEAT_NODE_ADDR_OFFSET	10
#define HEARTBEAT_NODE_TYPE_BITS	10
#define HEARTBEAT_NODE_TYPE_OFFSET      0

#define HEARTBEAT_LAST_SCANDAL_ERROR_BYTE        0
#define HEARTBEAT_LAST_USER_ERROR_BYTE           1
#define HEARTBEAT_SCVERSION_BYTE                 2
#define HEARTBEAT_NUMERRORS_BYTE                 3

/*
Heartbeat consists of the following:
	ID: 	3 Bits Priority
			8 Bits Message type
			8 Bits Node address
			10 Bits Node type
	Data:	8 bits last scandal error
	        8 bits last user error
		8 bits number of errors
		8 bits scandal version
		32 bits time
*/

/* Reset messages */
#define RESET_NODE_ADDR_BITS		8
#define RESET_NODE_ADDR_OFFSET		10

/* Scandal error messages */
#define SCANDAL_ERROR_NODE_ADDR_BITS	8
#define SCANDAL_ERROR_NODE_ADDR_OFFSET	10
#define SCANDAL_ERROR_NODE_TYPE_BITS	10
#define SCANDAL_ERROR_NODE_TYPE_OFFSET  0

/* User error messages */
#define USER_ERROR_NODE_ADDR_BITS	8
#define USER_ERROR_NODE_ADDR_OFFSET	10
#define USER_ERROR_NODE_TYPE_BITS	10
#define USER_ERROR_NODE_TYPE_OFFSET     0

/* Command messages */
#define COMMAND_DEST_ADDR_BITS	        8 
#define COMMAND_DEST_ADDR_OFFSET	10

#define COMMAND_NUM_BITS                10
#define COMMAND_NUM_OFFSET              0

/* Message specific #defines */

/* Configuration */
	/* Configuration parameters */
#define CONFIG_ADDR 			0	/* Data: 8 bits of address */
#define CONFIG_IN_CHAN_SOURCE 		1	/* Data: 16 bits channel number, 8 bits source node, 16 bits source num */
#define CONFIG_OUT_CHAN_M		2	/* Data: 16 bits channel number, 32 bits new M */
#define CONFIG_OUT_CHAN_B		3	/* Data: 16 bits channel number, 32 bits new B */

/* Utility macros for manipulating messages */
#define SCANDAL_MSG_PRIORITY(msg)         ((msg->id >> PRI_OFFSET) & ((1<<PRI_BITS) -1))
#define SCANDAL_MSG_TYPE(msg)              ((msg->id >> TYPE_OFFSET) & ((1<<TYPE_BITS) - 1))

#define FIRST_32_BITS(msg) ((((u32)((msg)->data[0] & 0xFF)) << 24) |\
					       (((u32)((msg)->data[1] & 0xFF)) << 16) |\
					       (((u32)((msg)->data[2] & 0xFF)) << 8) |\
					       (((u32)((msg)->data[3] & 0xFF)) << 0))

#define SECOND_32_BITS(msg) ((((u32)msg->data[4] & 0xFF) << 24) |\
					       (((u32)msg->data[5] & 0xFF) << 16) |\
					       (((u32)msg->data[6] & 0xFF) << 8) |\
					       (((u32)msg->data[7] & 0xFF) << 0))


/* Channel message */
#define SCANDAL_CHANNEL_MSG_ADDR(msg)      ((msg->id >> CHANNEL_SOURCE_ADDR_OFFSET) &\
					    ((1<<CHANNEL_SOURCE_ADDR_BITS) - 1))
#define SCANDAL_CHANNEL_MSG_CHAN_NUM(msg)  ((msg->id >> CHANNEL_NUM_OFFSET) &\
					    ((1<<CHANNEL_NUM_BITS) - 1))
#define SCANDAL_CHANNEL_MSG_VALUE(msg)     ((((u32)((msg)->data[0] & 0xFF)) << 24) |\
					    (((u32)((msg)->data[1] & 0xFF)) << 16) |\
					    (((u32)((msg)->data[2] & 0xFF)) << 8) |\
					    (((u32)((msg)->data[3] & 0xFF))))
#define SCANDAL_CHANNEL_MSG_TIME(msg)     ((((u32)(msg)->data[4] & 0xFF) << 24) |\
					    (((u32)((msg)->data[5] & 0xFF) << 16) |\
					    (((u32)(msg)->data[6] & 0xFF) << 8) |\
					    (((u32)(msg)->data[7] & 0xFF) << 0)))

/* Heartbeat message */
#define SCANDAL_HEARTBEAT_MSG_ADDR(msg)    ((msg->id >> HEARTBEAT_NODE_ADDR_OFFSET) &\
					    ((1<<HEARTBEAT_NODE_ADDR_BITS) - 1))
#define SCANDAL_HEARTBEAT_MSG_NODETYPE(msg) ((msg->id >> HEARTBEAT_NODE_TYPE_OFFSET) &\
					     ((1<<HEARTBEAT_NODE_TYPE_BITS) - 1))
#define SCANDAL_HEARTBEAT_MSG_LAST_SCANDAL_ERROR(msg) (msg->data[HEARTBEAT_LAST_SCANDAL_ERROR_BYTE])
#define SCANDAL_HEARTBEAT_MSG_LAST_USER_ERROR(msg) (msg->data[HEARTBEAT_LAST_USER_ERROR_BYTE])
#define SCANDAL_HEARTBEAT_MSG_SCVERSION(msg) (msg->data[HEARTBEAT_SCVERSION_BYTE])
#define SCANDAL_HEARTBEAT_MSG_NUMERRORS(msg) (msg->data[HEARTBEAT_NUMERRORS_BYTE])
#define SCANDAL_HEARTBEAT_MSG_TIME(msg)     ((((u32)msg->data[4] & 0xFF) << 24) |\
					     (((u32)msg->data[5] & 0xFF) << 16) |\
					     (((u32)msg->data[6] & 0xFF) << 8) |\
					     (((u32)msg->data[7] & 0xFF) << 0))

/* Command message */ 
#define SCANDAL_COMMAND_MSG_ADDR(msg)    ((msg->id >> COMMAND_DEST_ADDR_OFFSET) &\
					    ((1<< COMMAND_DEST_ADDR_BITS) - 1))

					      
/* Defaults*/
#define DEFAULT_M			1000
#define DEFAULT_B			0

/* Frame definition structures */

/*typedef struct channel_id{
	unsigned int padding:3;
	unsigned int priority:3;
	unsigned int type:8;
	unsigned int source:8;
	unsigned int channel_num:10;
}sc_channel_id; */

typedef u32 sc_channel_id;
typedef struct channel_data {
	u32 value;
 	u32 time;
} sc_channel_data;

typedef struct channel_frame {
	sc_channel_id id;
	sc_channel_data data;
	u32 length;
} sc_channel_frame;

/* Function Prototypes */
u08 			scandal_init(void);
s32 			scandal_get_m(u16 chan_num);
s32 			scandal_get_b(u16 chan_num);
void			scandal_set_m(u16 chan_num, s32 value);
void			scandal_set_b(u16 chan_num, s32 value);

s32 			scandal_get_in_channel_value(u16 chan_num);
sc_time_t 		scandal_get_in_channel_time(u16 chan_num);
sc_time_t 		scandal_get_in_channel_rcvd_time(u16 chan_num);
u08 			scandal_in_channel_is_valid(u16 chan_num);
in_channel*		scandal_get_in_channel(u16 chan_num);

typedef			void (*in_channel_handler)(int32_t value, uint32_t src_time);
void			scandal_register_in_channel_handler(int chan_num, in_channel_handler handler);

u08 			scandal_get_addr(void);
u32 			scandal_get_mac(void);
u32 			scandal_get_time(void);

void 			handle_scandal(void);

#endif
