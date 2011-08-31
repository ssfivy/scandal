/* --------------------------------------------------------------------------
	Scandal Message
	File name: scandal_message.c
	Author: David Snowdon

    Copyright (C) David Snowdon, 2009. 
 
	Date: 20/9/03
   -------------------------------------------------------------------------- */

#include <scandal/types.h>
#include <scandal/can.h>
#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/timer.h>
#include <scandal/error.h>

#include <project/scandal_config.h>

static inline u08 scandal_build_channel_msg(can_msg* msg, 
			u08 pri, u08 chan_num, u32 value, sc_time_t timestamp) {
	/* Load up the first four bytes with the value */
	msg->data[0] = (value >> 24) & 0xFF;
	msg->data[1] = (value >> 16) & 0xFF;
	msg->data[2] = (value >> 8) & 0xFF;
	msg->data[3] = (value >> 0) & 0xFF;

	msg->id = scandal_mk_channel_id(pri, scandal_get_addr(), chan_num);

	/* Load up the second four bytes with the time */
	msg->data[4] = (timestamp >> 24) & 0xFF;
	msg->data[5] = (timestamp >> 16) & 0xFF;
	msg->data[6] = (timestamp >> 8) & 0xFF;
	msg->data[7] = (timestamp >> 0) & 0xFF;

	msg->length = 8;

	return NO_ERR;
}

u08 scandal_send_channel_with_timestamp(u08 pri, u16 chan_num, 
			u32 value, sc_time_t timestamp) {
	can_msg msg;

	scandal_build_channel_msg(&msg, pri, chan_num, value, timestamp);

	if(can_send_msg(&msg, 1) != NO_ERR){
		/*! \todo Do something intelligent when an error occurs */
		// We need to do something here because something went wrong!!
	}

	/*! \todo Return reasonable error codes */
	return NO_ERR;
}

u08 scandal_build_heartbeat_msg(can_msg* msg, u08 last_scandal_error,
			u08 last_user_error, u08 scandal_version, u08 num_errors) {
	u32 value;

	value =  scandal_get_realtime32();
	msg->id = scandal_mk_heartbeat_id( );

	msg->data[4] = (value >> 24) & 0xFF;
	msg->data[5] = (value >> 16) & 0xFF;
	msg->data[6] = (value >> 8) & 0xFF;
	msg->data[7] = (value >> 0) & 0xFF;

	/* Load up the first few bytes */
	msg->data[HEARTBEAT_LAST_SCANDAL_ERROR_BYTE] = last_scandal_error;
	msg->data[HEARTBEAT_LAST_USER_ERROR_BYTE] = last_user_error;
	msg->data[HEARTBEAT_SCVERSION_BYTE] = scandal_version;
	msg->data[HEARTBEAT_NUMERRORS_BYTE] = num_errors;

	msg->length = 8;

	return NO_ERR; 
}

u08 scandal_send_heartbeat(u32 status) {
	can_msg msg;

	scandal_build_heartbeat_msg(&msg,
				scandal_get_last_scandal_error(), 
				scandal_get_last_user_error(), 
				SCANDAL_VERSION, 
				scandal_get_num_errors());

	if(can_send_msg(&msg, 1) != NO_ERR){
		/*! \todo Do something intelligent when an error occurs */
		// We need to do something here because something went wrong!!
	}

	/*! \todo Return reasonable error codes */
	return NO_ERR;
}

u08 scandal_send_scandal_error(u08 err) {
	u32 value;
	can_msg msg;

	value= scandal_get_realtime32();
	msg.id = scandal_mk_scandal_error_id();

	msg.data[0] = err;

	msg.data[4] = (value >> 24) & 0xFF;
	msg.data[5] = (value >> 16) & 0xFF;
	msg.data[6] = (value >> 8) & 0xFF;
	msg.data[7] = (value >> 0) & 0xFF;
	msg.length = 8;
	
	if(can_send_msg(&msg, 1) != NO_ERR){
		/*! \todo Do something intelligent when an error occurs */
		// We need to do something here because something went wrong!!
	}
	return NO_ERR;
}

u08 scandal_send_user_error(u08 err){
	u32 value;
	can_msg msg;

	value = scandal_get_realtime32();
	msg.id = scandal_mk_user_error_id();

	msg.data[0] = err;

	msg.data[4] = (value >> 24) & 0xFF;
	msg.data[5] = (value >> 16) & 0xFF;
	msg.data[6] = (value >> 8) & 0xFF;
	msg.data[7] = (value >> 0) & 0xFF;
	msg.length = 8;

	if(can_send_msg(&msg, 1) != NO_ERR){
		/*! \todo Do something intelligent when an error occurs */
		// We need to do something here because something went wrong!!
	}
	return NO_ERR;
}

u08 scandal_send_reset(u08 priority, u08 node) {
	can_msg msg;
  
	msg.id = scandal_mk_reset_id(priority, node);
	msg.length = 8;

	if(can_send_msg(&msg, 1) != NO_ERR){
		/*! \todo Do something intelligent when an error occurs */
		// We need to do something here because something went wrong!!
	}
	return NO_ERR;

}

u08 scandal_send_user_config(u08 priority, u08 node, u08 param, 
			u32 value1, u32 value2) {
	u32 value;
	can_msg msg;

	value = scandal_get_realtime32();
	msg.id = scandal_mk_user_config_id(priority, node, param);

	msg.data[0] = (value1 >> 24) & 0xFF;
	msg.data[1] = (value1 >> 16) & 0xFF;
	msg.data[2] = (value1 >> 8) & 0xFF;
	msg.data[3] = (value1 >> 0) & 0xFF;

	msg.data[4] = (value2 >> 24) & 0xFF;
	msg.data[5] = (value2 >> 16) & 0xFF;
	msg.data[6] = (value2 >> 8) & 0xFF;
	msg.data[7] = (value2 >> 0) & 0xFF;
	msg.length = 8;

	if(can_send_msg(&msg, 1) != NO_ERR){
		/*! \todo Do something intelligent when an error occurs */
		// We need to do something here because something went wrong!!
	}

	return NO_ERR;
}

u08 scandal_send_timesync(u08 priority, u08 node, uint64_t newtime) {
    can_msg msg;
    uint32_t val;

    msg.id = scandal_mk_timesync_id(priority); 

    val = (newtime >> 32) & 0xFFFFFFFF;

    msg.data[0] = (val >> 24) & 0xFF; 
    msg.data[1] = (val >> 16) & 0xFF; 
    msg.data[2] = (val >> 8) & 0xFF; 
    msg.data[3] = (val >> 0) & 0xFF;

    msg.data[4] = (newtime >> 24) & 0x00000000000000FF;;
    msg.data[5] = (newtime >> 16) & 0x00000000000000FF;;
    msg.data[6] = (newtime >> 8) & 0x00000000000000FF;;
    msg.data[7] = (newtime >> 0) & 0x00000000000000FF;;
    msg.length = 8; 

	if(can_send_msg(&msg, 0) != NO_ERR){
		/*! \todo Do something intelligent when an error occurs */
		// We need to do something here because something went wrong!!
	}

	return NO_ERR;
}
