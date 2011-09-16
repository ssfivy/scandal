#include <scandal/tritium.h>
#include <scandal/can.h>
#include <scandal/engine.h>

typedef			void (*ws_bus_callback)(float bus_current, float bus_voltage, uint32_t time);
void			scandal_register_ws_bus_callback(ws_bus_callback cb);

typedef			void (*ws_temp_callback)(float hs_temp, float motor_temp, uint32_t time);
void			scandal_register_ws_temp_callback(ws_temp_callback cb);

void scandal_handle_ws_message(can_msg *msg);
