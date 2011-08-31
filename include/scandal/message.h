/* --------------------------------------------------------------------------
	Scandal Message
	File name: scandal_message.h
	Author: David Snowdon

	Date: 20/9/03
 
    Copyright (C) 2003 David Snowdon. 
   -------------------------------------------------------------------------- */

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

#ifndef __SCANDAL_MESSAGE__
#define __SCANDAL_MESSAGE__

#include <scandal/types.h>
#include <scandal/engine.h>

#include <project/scandal_config.h>

/* Priority definitions */
#define CRITICAL_PRIORITY		0
#define NETWORK_HIGH			1
#define CONTROL_HIGH			2
#define TELEM_HIGH			3
#define CONTROL_LOW			5
#define NETWORK_LOW			6
#define TELEM_LOW			7

/* Message type definitions */
#define CHANNEL_TYPE  	                0
#define CONFIG_TYPE			1
#define HEARTBEAT_TYPE 	                2
#define SCANDAL_ERROR_TYPE		3
#define USER_ERROR_TYPE                 4
#define RESET_TYPE			5                
#define USER_CONFIG_TYPE		6 
#define COMMAND_TYPE			7

/* Fields common to all frames */
#define PRI_BITS		        3
#define PRI_OFFSET		        26
#define MSG_PRIORITY(msg)               ((msg->id >> PRI_OFFSET) & ((1<<PRI_BITS) -1))

#define TYPE_BITS		        8
#define TYPE_OFFSET		        18
#define MSG_TYPE(msg)                   ((msg->id >> TYPE_OFFSET) & ((1<<TYPE_BITS) - 1))



/* Channel messages */
#define CHANNEL_SOURCE_ADDR_BITS	8
#define CHANNEL_SOURCE_ADDR_OFFSET	10
#define CHANNEL_MSG_ADDR(msg)           ((msg->id >> CHANNEL_SOURCE_ADDR_OFFSET) &\
					 ((1<<CHANNEL_SOURCE_ADDR_BITS) - 1))

#define CHANNEL_NUM_BITS	        10
#define CHANNEL_NUM_OFFSET  	        0
#define CHANNEL_MSG_CHAN_NUM(msg)       ((msg->id >> CHANNEL_NUM_OFFSET) &\
					 ((1<<CHANNEL_NUM_BITS) - 1))

#define CHANNEL_MSG_VALUE(msg)          ((((u32)((msg)->data[0] & 0xFF)) << 24) |\
					 (((u32)((msg)->data[1] & 0xFF)) << 16) |\
					 (((u32)((msg)->data[2] & 0xFF)) << 8) |\
					 (((u32)((msg)->data[3] & 0xFF))))
#define CHANNEL_MSG_TIME(msg)           ((((u32)(msg)->data[4] & 0xFF) << 24) |\
					 (((u32)((msg)->data[5] & 0xFF) << 16) |\
					 (((u32)(msg)->data[6] & 0xFF) << 8) |\
					 (((u32)(msg)->data[7] & 0xFF) << 0)))




/* Configuration messages (both user and scandal) */
#define CONFIG_NODE_ADDR_BITS	        8
#define CONFIG_NODE_ADDR_OFFSET	        10
#define CONFIG_MSG_ADDR(msg)            ((msg->id >> CONFIG_NODE_ADDR_OFFSET) &\
					 ((1<<CONFIG_NODE_ADDR_BITS) - 1))

#define CONFIG_PARAM_BITS	        8
#define CONFIG_PARAM_OFFSET	        0
#define CONFIG_MSG_PARAM(msg)           ((msg->id >> CONFIG_PARAM_OFFSET) &\
					 ((1<<CONFIG_PARAM_BITS) - 1))



/* Heartbeat messages */
#define HEARTBEAT_NODE_ADDR_BITS	8
#define HEARTBEAT_NODE_ADDR_OFFSET	10
#define HEARTBEAT_MSG_NODE_ADDR(msg)    ((msg->id >> HEARTBEAT_NODE_ADDR_OFFSET) &\
					 ((1<<HEARTBEAT_NODE_ADDR_BITS) - 1))

#define HEARTBEAT_NODE_TYPE_BITS	10
#define HEARTBEAT_NODE_TYPE_OFFSET      0
#define HEARTBEAT_MSG_NODE_TYPE(msg)    ((msg->id >> HEARTBEAT_NODE_TYPE_OFFSET) &\
					 ((1<<HEARTBEAT_NODE_TYPE_BITS) - 1))

#define HEARTBEAT_LAST_SCANDAL_ERROR_BYTE        0
#define HEARTBEAT_MSG_LAST_SCANDAL_ERROR(msg)    (msg->data[HEARTBEAT_LAST_SCANDAL_ERROR_BYTE])

#define HEARTBEAT_LAST_USER_ERROR_BYTE           1
#define HEARTBEAT_MSG_LAST_USER_ERROR(msg)       (msg->data[HEARTBEAT_LAST_USER_ERROR_BYTE])

#define HEARTBEAT_SCVERSION_BYTE                 2
#define HEARTBEAT_MSG_SCVERSION(msg)             (msg->data[HEARTBEAT_SCVERSION_BYTE])

#define HEARTBEAT_NUMERRORS_BYTE                 3
#define HEARTBEAT_MSG_NUMERRORS(msg)             (msg->data[HEARTBEAT_NUMERRORS_BYTE])

#define HEARTBEAT_MSG_TIME(msg)     ((((u32)msg->data[4] & 0xFF) << 24) |\
					     (((u32)msg->data[5] & 0xFF) << 16) |\
					     (((u32)msg->data[6] & 0xFF) << 8) |\
					     (((u32)msg->data[7] & 0xFF) << 0))


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
#define RESET_MSG_NODE_ADDR(msg)        ((msg->id >> RESET_NODE_ADDR_OFFSET) &\
					 ((1<<RESET_NODE_ADDR_BITS) - 1))


/* Scandal error messages */
#define SCANDAL_ERROR_NODE_ADDR_BITS	8
#define SCANDAL_ERROR_NODE_ADDR_OFFSET	10
#define SCANDAL_ERROR_MSG_NODE_ADDR(msg) ((msg->id >> SCANDAL_ERROR_NODE_ADDR_OFFSET) &\
					 ((1<<SCANDAL_ERROR_NODE_ADDR_BITS) - 1))


#define SCANDAL_ERROR_NODE_TYPE_BITS	10
#define SCANDAL_ERROR_NODE_TYPE_OFFSET  0
#define SCANDAL_ERROR_MSG_NODE_TYPE(msg) ((msg->id >> SCANDAL_ERROR_NODE_TYPE_OFFSET) &\
					 ((1<<SCANDAL_ERROR_NODE_TYPE_BITS) - 1))


/* User error messages */
#define USER_ERROR_NODE_ADDR_BITS	8
#define USER_ERROR_NODE_ADDR_OFFSET	10
#define USER_ERROR_MSG_NODE_ADDR(msg)   ((msg->id >> USER_ERROR_NODE_ADDR_OFFSET) &\
					 ((1<<USER_ERROR_NODE_ADDR_BITS) - 1))

#define USER_ERROR_NODE_TYPE_BITS	10
#define USER_ERROR_NODE_TYPE_OFFSET     0
#define USER_ERROR_MSG_NODE_ADDR(msg)   ((msg->id >> USER_ERROR_NODE_ADDR_OFFSET) &\
					 ((1<<USER_ERROR_NODE_ADDR_BITS) - 1))



/* Configuration parameters */
#define CONFIG_ADDR 			0	/* Data: 8 bits of address */
#define CONFIG_IN_CHAN_SOURCE 		1	/* Data: 16 bits channel number, 8 bits source node, 16 bits source num */
#define CONFIG_OUT_CHAN_M		2	/* Data: 16 bits channel number, 32 bits new M */
#define CONFIG_OUT_CHAN_B		3	/* Data: 16 bits channel number, 32 bits new B */


/* Generic utility macros */
#define FIRST_32_BITS(msg) ((((u32)((msg)->data[0] & 0xFF)) << 24) |\
					       (((u32)((msg)->data[1] & 0xFF)) << 16) |\
					       (((u32)((msg)->data[2] & 0xFF)) << 8) |\
					       (((u32)((msg)->data[3] & 0xFF)) << 0))

#define SECOND_32_BITS(msg) ((((u32)msg->data[4] & 0xFF) << 24) |\
					       (((u32)msg->data[5] & 0xFF) << 16) |\
					       (((u32)msg->data[6] & 0xFF) << 8) |\
					       (((u32)msg->data[7] & 0xFF) << 0))


/* Heartbeat message */
#define SCANDAL_HEARTBEAT_MSG_ADDR(msg)    ((msg->id >> HEARTBEAT_NODE_ADDR_OFFSET) &\
					    ((1<<HEARTBEAT_NODE_ADDR_BITS) - 1))
#define SCANDAL_HEARTBEAT_MSG_NODETYPE(msg) ((msg->id >> HEARTBEAT_NODE_TYPE_OFFSET) &\
					     ((1<<HEARTBEAT_NODE_TYPE_BITS) - 1))
#define SCANDAL_HEARTBEAT_MSG_LAST_SCANDAL_ERROR(msg) (msg->data[HEARTBEAT_LAST_SCANDAL_ERROR_BYTE])
#define SCANDAL_HEARTBEAT_MSG_LAST_USER_ERROR(msg) (msg->data[HEARTBEAT_LAST_USER_ERROR_BYTE])
#define SCANDAL_HEARTBEAT_MSG_SCVERSION(msg) (msg->data[HEARTBEAT_SCVERSION_BYTE])
#define SCANDAL_HEARTBEAT_MSG_NUMERRORS(msg) (msg->data[HEARTBEAT_NUMERRORS_BYTE])
					      


/* ID constructors */ 
static inline u32 scandal_mk_channel_id(u08 priority, u08 source, u16 channel_num){
	return( ((u32)(priority & 0x07) << PRI_OFFSET) |
		((u32)CHANNEL_TYPE << TYPE_OFFSET) |
		((u32)(source & 0xFF) << CHANNEL_SOURCE_ADDR_OFFSET) |
		((u32)(channel_num & 0x03FF) << CHANNEL_NUM_OFFSET));
}

static inline u32	scandal_mk_config_id(u08 priority, u08 node, u08 parameter){
	return( ((u32)(priority & 0x07) << PRI_OFFSET) |
		((u32)CONFIG_TYPE << TYPE_OFFSET) |
		((u32)(node & 0xFF) << CONFIG_NODE_ADDR_OFFSET) |
		((u32)(parameter & 0x03FF) << CONFIG_PARAM_OFFSET ));
}

static inline u32	scandal_mk_heartbeat_id(){
	return( ((u32)(NETWORK_LOW & 0x07) << PRI_OFFSET) |
		((u32)HEARTBEAT_TYPE << TYPE_OFFSET) |
		((u32)(scandal_get_addr() & 0xFF) << HEARTBEAT_NODE_ADDR_OFFSET) |
		((u32)THIS_DEVICE_TYPE << HEARTBEAT_NODE_TYPE_OFFSET));
}

static inline u32	scandal_mk_reset_id(u08 priority, u08 node){
	return( ((u32)(priority & 0x07) << PRI_OFFSET) |
		((u32)RESET_TYPE << TYPE_OFFSET) |
		((u32)(node & 0xFF) << RESET_NODE_ADDR_OFFSET));
}

static inline u32	scandal_mk_scandal_error_id(){
	return( ((u32)(NETWORK_HIGH & 0x07) << PRI_OFFSET) |
		((u32)SCANDAL_ERROR_TYPE << TYPE_OFFSET) |
		((u32)(scandal_get_addr() & 0xFF) << SCANDAL_ERROR_NODE_ADDR_OFFSET) |
		((u32)THIS_DEVICE_TYPE << SCANDAL_ERROR_NODE_TYPE_OFFSET));
}

static inline u32	scandal_mk_user_error_id(){
	return( ((u32)(NETWORK_HIGH & 0x07) << PRI_OFFSET) |
		((u32)USER_ERROR_TYPE << TYPE_OFFSET) |
		((u32)(scandal_get_addr() & 0xFF) << USER_ERROR_NODE_ADDR_OFFSET) |
		((u32)THIS_DEVICE_TYPE << USER_ERROR_NODE_TYPE_OFFSET));
}

static inline u32	scandal_mk_user_config_id(u08 priority, u08 node, u08 parameter){
	return( ((u32)(priority & 0x07) << PRI_OFFSET) |
		((u32)USER_CONFIG_TYPE << TYPE_OFFSET) |
		((u32)(node & 0xFF) << CONFIG_NODE_ADDR_OFFSET) |
		((u32)(parameter & 0x03FF) << CONFIG_PARAM_OFFSET ));
}

static inline u32 scandal_mk_command_id(u08 priority, u08 dest, u16 command_num){
	return( ((u32)(priority & 0x07) << PRI_OFFSET) |
		((u32)COMMAND_TYPE << TYPE_OFFSET) |
		((u32)(dest & 0xFF) << COMMAND_DEST_ADDR_OFFSET) |
		((u32)(command_num & 0x03FF) << COMMAND_NUM_OFFSET));
}

static inline u32 scandal_mk_timesync_id(u08 priority){
	return( ((u32)(priority & 0x07) << PRI_OFFSET) |
		((u32)TIMESYNC_TYPE << TYPE_OFFSET));
}





/* Function prototypes */
u08 		    scandal_send_heartbeat(u32 status);
u08             scandal_send_channel_with_timestamp(u08 priority, u16 chan_num, 
						    u32 value, sc_time_t timestamp);
u08 			scandal_send_scandal_error(u08 err);
u08 			scandal_send_user_error(u08 err);
u08             scandal_send_reset(u08 priority, u08 node);
u08             scandal_send_user_config(u08 priority, u08 node, u08 param, u32 value1, u32 value2);
u08             scandal_send_timesync(u08 priority, u08 node, uint64_t newtime);

/* More static inline functions */ 

static inline u08 
scandal_send_channel(u08 pri, u16 chan_num, u32 value){
  sc_time_t timestamp = scandal_get_realtime32(); 
  return scandal_send_channel_with_timestamp(pri, chan_num, value, timestamp); 
}

#endif
