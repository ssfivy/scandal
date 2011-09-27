#include <scandal/tritium.h>
#include <scandal/can.h>
#include <scandal/engine.h>

typedef			void (*ws_base_callback)(char *tritium_id, uint32_t serial_number, uint32_t time);
void			scandal_register_ws_base_callback(ws_base_callback cb);

typedef			void (*ws_status_callback)(uint8_t rcv_err_count, uint8_t tx_err_count, uint16_t active_motor,
						uint16_t err_flags, uint16_t limit_flags, uint32_t time);
void			scandal_register_ws_status_callback(ws_status_callback cb);

typedef			void (*ws_bus_callback)(float bus_current, float bus_voltage, uint32_t time);
void			scandal_register_ws_bus_callback(ws_bus_callback cb);

typedef			void (*ws_velocity_callback)(float vehicle_velocity, float motor_rpm, uint32_t time);
void			scandal_register_ws_velocity_callback(ws_velocity_callback cb);

typedef			void (*ws_temp_callback)(float hs_temp, float motor_temp, uint32_t time);
void			scandal_register_ws_temp_callback(ws_temp_callback cb);

void scandal_handle_ws_message(can_msg *msg);
