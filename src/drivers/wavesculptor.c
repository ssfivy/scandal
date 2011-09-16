#include <scandal/engine.h>
#include <scandal/timer.h>
#include <scandal/wavesculptor.h>
#include <scandal/tritium.h>
#include <scandal/can.h>
#include <scandal/stdio.h>

ws_bus_callback   ws_bus_handler_cb = 0;
ws_temp_callback   ws_temp_handler_cb = 0;

void scandal_register_ws_bus_callback(ws_bus_callback cb) {
	can_register_id(CAN_ID_STD_MASK, MC_BUS, 0, CAN_STD_MSG);
	ws_bus_handler_cb = cb;
}

void scandal_register_ws_temp_callback(ws_temp_callback cb) {
	can_register_id(CAN_ID_STD_MASK, MC_HEATSINK_MOTOR_TEMP, 0, CAN_STD_MSG);
	ws_temp_handler_cb = cb;
}

void scandal_handle_ws_message(can_msg *msg) {
	switch(msg->id) {
	 case MC_BUS:
		if (ws_bus_handler_cb != 0) {
			group_64 *data = (group_64 *)msg->data;
			ws_bus_handler_cb(data->data_fp[1], data->data_fp[0], sc_get_timer());
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
