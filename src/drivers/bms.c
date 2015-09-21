#include <scandal/engine.h>
#include <scandal/timer.h>
#include <scandal/bms.h>
#include <scandal/tritium.h>
#include <scandal/can.h>
#include <scandal/stdio.h>
#include <scandal/error.h> //Added
#include <scandal/message.h>

uint8_t scandal_store_bms_message(can_msg *msg, BMS_Output_Struct *dataStruct){
	uint8_t checks = NO_ERR;
	uint16_t baseAddress = msg->id & 0x7E0;

	/* Make sure the message is NOT extended as this is not a WS message */
    if (msg->ext){
        checks = EXT_ID_ERR; //Extended ID when we weren't expecting one
    }

    /* Make certain we're storing information in the right place here */
    if (!(dataStruct->BaseAddress == baseAddress)){
        checks = NO_MSG_ERR; //Wrong or no message error
        UART_printf("WRONG BASE of %d\r\n", dataStruct->BaseAddress);
    }

    if (!checks) {
    	group_64 *data = (group_64 *)msg->data;

    	switch(msg->id) {

    		case BMS_BASE:
    			dataStruct->DeviceID = data->data_u32[0];
    			dataStruct->SerialNumber = data->data_u32[1];
    			break;

    		case BMS_PACK_SOC:
    			dataStruct->battery_consumed_Ah = data->data_fp[0];
    			dataStruct->battery_SOC_percent = data->data_fp[1];
    			break;

    		case BMS_BALANCE_SOC:
    			dataStruct->balance_consumed_Ah = data->data_fp[0];
    			dataStruct->balance_SOC_percent = data->data_fp[1];
    			break;

    		case BMS_CHARGER_TELEMETRY:
    			dataStruct->charge_cell_voltage_error = data->data_16[0];
    			dataStruct->cell_temp_margin = data->data_16[1];
    			dataStruct->discharge_cell_voltage_error = data->data_16[2];
    			dataStruct->total_pack_capacity = data->data_u16[3];
    			break;

    		case BMS_PRECHARGE_STATUS:
    			dataStruct->precharge_contactor_status = data->data_u8[0];
    			dataStruct->precharge_state = data->data_u8[1];
    			dataStruct->12V_contactor_supply = data->data_u16[1];
    			dataStruct->precharge_timer_status = data->data_u8[6];
    			dataStruct->precharge_timer_counter = data->data_u8[7];
    			break;

    		case BMS_MINMAX_CELL_VOLTAGE:
    			dataStruct->min_cell_voltage = data->data_u16[0];
    			dataStruct->max_cell_voltage = data->data_u16[1];
    			dataStruct->min_cell_voltage_CMU_ID = data->data_u8[4];
    			dataStruct->min_cell_voltage_ID = data->data_u8[5];
    			dataStruct->max_cell_voltage_CMU_ID = data->data_u8[6];
    			dataStruct->max_cell_voltage_ID = data->data_u8[7];
    			break;

    		case BMS_MINMAX_CELL_TEMP:
    			dataStruct->min_cell_temp = data->data_u16[0];
    			dataStruct->max_cell_temp = data->data_u16[1];
    			dataStruct->min_cell_temp_CMU_ID = data->data_u8[4];
    			dataStruct->max_cell_temp_CMU_ID = data->data_u8[6];
    			break;

    		case BMS_PACK_VOLTAGE_CURRENT:
    			dataStruct->battery_voltage = data->data_u32[0];
    			dataStruct->battery_current = data->data_32[1];
    			break;

    		case BMS_PACK_STATUS:
    			dataStruct->balance_voltage_threshold_rising = data->data_u16[0];
    			dataStruct->balance_voltage_threshold_falling = data->data_u16[1];
    			dataStruct->battery_status_flags = data->data_u8[4];
    			dataStruct->BMS_CMU_COUNT = data->data_u8[5];
    			dataStruct->BMU_firmware_build = data->data_u16[3];
    			break;

    		case BMS_PACK_FAN_STATUS:
    			dataStruct->FAN_SPEED_0 = data->data_u16[0];
    			dataStruct->FAN_SPEED_1 = data->data_u16[1];
    			dataStruct->current_consumption_aux = data->data_u16[2];
    			dataStruct->current_consumption_CMU = data->data_u16[3];
    			break;

    		case BMS_PACK_EXTENDED_STATUS:
    			dataStruct->extended_status_flags = data->data_32[0];
    			dataStruct->BMU_hardware_version = data->data_u8[4];
    			dataStruct->BMU_Model_ID = data->data_8[5];
    			break;

    	}
    }

    return checks;
}