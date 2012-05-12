#include <scandal/engine.h>
#include <scandal/timer.h>
#include <scandal/wavesculptor.h>
#include <scandal/tritium.h>
#include <scandal/can.h>
#include <scandal/stdio.h>

ws_base_callback ws_base_handler_cb = 0;
ws_status_callback ws_status_handler_cb = 0;
ws_bus_callback ws_bus_handler_cb = 0;
ws_velocity_callback ws_velocity_handler_cb = 0;
ws_temp_callback ws_temp_handler_cb = 0;

void scandal_register_ws_base_callback(ws_base_callback cb) {
	can_register_id(CAN_ID_STD_MASK, MC_BASE, 0, CAN_STD_MSG);
	ws_base_handler_cb = cb;
}

void scandal_register_ws_status_callback(ws_status_callback cb) {
	can_register_id(CAN_ID_STD_MASK, MC_STATUS, 0, CAN_STD_MSG);
	ws_status_handler_cb = cb;
}

void scandal_register_ws_bus_callback(ws_bus_callback cb) {
	can_register_id(CAN_ID_STD_MASK, MC_BUS, 0, CAN_STD_MSG);
	ws_bus_handler_cb = cb;
}

void scandal_register_ws_velocity_callback(ws_velocity_callback cb) {
	can_register_id(CAN_ID_STD_MASK, MC_VELOCITY, 0, CAN_STD_MSG);
	ws_velocity_handler_cb = cb;
}

void scandal_register_ws_temp_callback(ws_temp_callback cb) {
	can_register_id(CAN_ID_STD_MASK, MC_HEATSINK_MOTOR_TEMP, 0, CAN_STD_MSG);
	ws_temp_handler_cb = cb;
}

void scandal_handle_ws_message(can_msg *msg) {
	switch(msg->id) {
	 case MC_BASE:
		if (ws_base_handler_cb != 0) {
			group_64 *data = (group_64 *)msg->data;
			ws_base_handler_cb((char *)(data->data_u32[1]), data->data_u32[0],
						sc_get_timer());
		}
		break;

	 case MC_STATUS:
		if (ws_bus_handler_cb != 0) {
			group_64 *data = (group_64 *)msg->data;
			int8_t rcv_err_count =  data->data_u8[7];
			uint8_t tx_err_count =  data->data_u8[6];
			uint16_t active_motor = data->data_u16[2];
			uint16_t err_flags =    data->data_u16[1];
			uint16_t limit_flags =  data->data_u16[0];
			ws_status_handler_cb(rcv_err_count, tx_err_count, active_motor,
						err_flags, limit_flags, sc_get_timer());
		}
		break;

	 case MC_BUS:
		if (ws_bus_handler_cb != 0) {
			group_64 *data = (group_64 *)msg->data;
			ws_bus_handler_cb(data->data_fp[1], data->data_fp[0], sc_get_timer());
		}
		break;

	 case MC_VELOCITY:
		if (ws_velocity_handler_cb != 0) {
			group_64 *data = (group_64 *)msg->data;
			ws_velocity_handler_cb(data->data_fp[1], data->data_fp[0], sc_get_timer());
		}
		break;

	 case MC_HEATSINK_MOTOR_TEMP:
		if (ws_temp_handler_cb != 0) {
			group_64 *data = (group_64 *)msg->data;
			ws_temp_handler_cb(data->data_fp[1], data->data_fp[0], sc_get_timer());
		}
		break;
	}
}
