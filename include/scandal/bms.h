#include <scandal/tritium.h>
#include <scandal/can.h>
#include <scandal/engine.h>

typedef struct _BMS_Output_Struct{
	uint16_t	BaseAddress;			// Tritium CAN Base Address for BMU transmitted messages
	uint16_t	DriveAddress;			// Tritium base address from which control messages are accepted
	uint16_t 	ControlAddress;			// Tritium base address for EV driver controls
	sc_time_t 	LastUpdateTime;
	uint32_t	DeviceID;
	uint32_t	SerialNumber;
	float		battery_consumed_Ah;
	float		battery_SOC_percent;
	float		balance_consumed_Ah;
	float 		balance_SOC_percent;
	int16_t		charge_cell_voltage_error;
	int16_t		cell_temp_margin;
	int16_t		discharge_cell_voltage_error;
	uint16_t	total_pack_capacity;
	uint8_t		precharge_contactor_status;
	uint8_t		precharge_state;
	uint16_t	12V_Contactor_Supply;
	uint8_t		precharge_timer_elapsed;
	uint8_t		precharge_timer_counter;
	uint16_t 	min_cell_voltage;
	uint16_t	max_cell_voltage;
	uint8_t		min_cell_voltage_CMU_ID;
	uint8_t		min_cell_voltage_ID;
	uint8_t		max_cell_voltage_CMU_ID;
	uint8_t		max_cell_voltage_ID;
	uint16_t	min_cell_temp;
	uint16_t	max_cell_temp;
	uint8_t		min_cell_temp_CMU_ID;
	uint8_t		max_cell_temp_CMU_ID;
	uint32_t	battery_voltage;
	int32_t		battery_current;
	uint16_t	balance_voltage_threshold_rising;
	uint16_t	balance_voltage_threshold_falling;
	uint8_t		battery_status_flags;
	uint8_t		BMS_CMU_count;
	uint16_t	BMU_firmware_build;
	uint16_t	FAN_SPEED_0;
	uint16_t	FAN_SPEED_1;
	uint16_t	current_consumption_aux;
	uint16_t	current_consumption_CMU;
	uint32_t	extended_status_flags;
	uint8_t		BMU_hardware_version;
	uint8_t		BMU_model_ID;
} BMS_Output_Struct;

uint8_t scandal_store_BMS_message(can_msg *msg, BMS_Output_Struct *dataStruct);