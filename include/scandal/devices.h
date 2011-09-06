/* --------------------------------------------------------------------------
	Scandal Device Definitions

	File name: scandal_devices.c
	Author: David Snowdon
	Date: Thursday, 5 September 2002

    Copyright (C) David Snowdon 2009. 
 
	Defines the device IDs and channel numbers
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

/* -----------------------------------------------
	Devices
   -----------------------------------------------*/
#define CURRENTSENSOR						1
#define DCDC								2
#define DRIVERCONTROLS						3
#define DRIVERDISPLAY						4
#define SWITCHCARD							5
#define MOTORCONTROLLER						6
#define MPPTNG								7
#define CANSERIAL							8
#define SUPPORTCAR							9
#define HYDRA								10
#define	GPS									11
#define TEMPSENSOR							12
#define NEGATIVESUM							13
#define COURSEPROFILE						14
#define UQMINTERFACE						15
#define SWITCHCARD4							16
#define DATALOGGER							17
#define MININGSGL							18
#define MINING								19
#define MINING2								20
#define TILT								21
#define UNSWMPPTNG							22
#define GPSMOUSE							23
#define MSPLCD								24
#define MSPSWITCH							25
#define LHSCONTROLS							26
#define TYREMASTER							27
#define SCULPTORBRIDGE                                                  28
#define CURRENTINT                                                      29
#define MSPONEWIRE							30
#define SMARTDCDC							31
#define STEERINGWHEEL						32
#define GPSBAROMETER                        33
#define TEMPLATE							34

/* -----------------------------------------------
	Channel Numbering Constants
   ----------------------------------------------- */
   
/* Driver Display */
	/* Out-Channels */
		#define DRIVERDISPLAY_AMBIENT         			0
		#define DRIVERDISPLAY_REF_NODE_CURRENT			1
		#define DRIVERDISPLAY_NUM_OUT_CHANNELS		        2
	/* In-Channels */
		#define DRIVERDISPLAY_SCROLL_UP				0
		#define DRIVERDISPLAY_SCROLL_DOWN			1
		#define DRIVERDISPLAY_MESSAGE				2
                #define DRIVERDISPLAY_BACKLIGHT                         3
		#define DRIVERDISPLAY_NUM_IN_CHANNELS			14
			/* Note, the driver display has more in-channels than this,
			   but the rest of them are dynamic and are defined in the 
			   driver display code */

/* Current Sensor */
	/* Out-Channels */
		#define CURRENTSENSOR_CURRENT				0
		#define CURRENTSENSOR_SENSOR_CURRENT			1
		#define CURRENTSENSOR_AMBIENT_TEMP			2
		#define CURRENTSENSOR_CURRENT_INT			3
		#define CURRENTSENSOR_POWER_INT				4
                #define CURRENTSENSOR_POWER                             5
		#define CURRENTSENSOR_NUM_OUT_CHANNELS			6
	/* In-Channels: None */
		#define CURRENTSENSOR_DISABLE				0
		#define CURRENTSENSOR_VOLTAGE_IN			1
		#define CURRENTSENSOR_SET_CURRENT_INT			2
		#define CURRENTSENSOR_SET_POWER_INT			3
		#define CURRENTSENSOR_NUM_IN_CHANNELS			4

/* DCDC Converter */
	/* Out Channels */
		#define DCDC_BATTERY_V 						0
		#define DCDC_HV_CURRENT_SENSE				1
		#define DCDC_UNPRO_BAT_MEAS					2
		#define DCDC_CHASSIS_TEMP					3
		#define DCDC_V12_MEASURE					4
		#define DCDC_V5_MEASURE						5
		#define DCDC_V12_UNPRO_MEASURE				6
		#define	DCDC_V5_UNPRO_MEASURE				7
		#define DCDC_V12_CURRENT					8
		#define DCDC_V5_CURRENT						9
		#define DCDC_AMBIENT_TEMP					10
		#define DCDC_NUM_OUT_CHANNELS				11
	/* In Channels */
		#define DCDC_INTERNAL_FAN					0
		#define DCDC_EXTERNAL_OUT					1
		#define DCDC_DISABLE_12V					2
		#define DCDC_DISABLE_5V						3
		#define DCDC_NUM_IN_CHANNELS				4

/* Driver Controls */
	/* Out CHannels */
		#define DRIVERCONTROLS_HORN_CHANNEL			0
		#define DRIVERCONTROLS_LEFT_IND_CHANNEL		1
		#define DRIVERCONTROLS_RIGHT_IND_CHANNEL	2
		#define DRIVERCONTROLS_CAMERA_CHANNEL		3
		#define DRIVERCONTROLS_BUTTON1_CHANNEL		4
		#define DRIVERCONTROLS_BUTTON2_CHANNEL		5
		#define DRIVERCONTROLS_BUTTON3_CHANNEL		6
		#define DRIVERCONTROLS_NUM_OUT_CHANNELS		7
	/* In Channels: None */
		#define DRIVERCONTROLS_NUM_IN_CHANNELS		0

/* Switch Card */
	/* Out Channels */
		#define SWITCHCARD_CURRENT_1				0
		#define SWITCHCARD_VOLTAGE_1				1
		#define SWITCHCARD_CURRENT_2				2
		#define SWITCHCARD_VOLTAGE_2				3
		#define SWITCHCARD_AMBIENT				4
		#define SWITCHCARD_REFNODE_CURRENT			5
		#define SWITCHCARD_CHANNEL_1_STATUS			6
		#define SWITCHCARD_CHANNEL_2_STATUS			7
		#define SWITCHCARD_NUM_OUT_CHANNELS			8

	/* In Channels */
		#define SWITCHCARD_CHANNEL_1				0
		#define SWITCHCARD_CHANNEL_2				1
                #define SWITCHCARD_NUM_IN_CHANNELS			2

/* Tritium Motor Controller */
	/* Out Channels */
		#define 	MC_ACTUALPWM					0
		#define 	MC_ACTUALI						1
		#define 	MC_ACTUALVEL					2
		#define 	MC_BUSV							3
		#define 	MC_CONTROLLERI					4
		#define 	MC_HEATSINKTEMP					5
		#define 	MC_MOTORTEMP					6
		#define 	MC_CONTROLLERTEMP				7
		#define 	MC_SMPSTEMP						8
		#define 	MC_15V							9
		#define 	MC_HORN							10
		#define 	MC_BRAKE						11
		#define 	MC_RIGHTINDICATOR				12
		#define 	MC_LEFTINDICATOR				13    
		#define 	MC_AUXOUT1						14
		#define		MC_AUXOUT2						15  
		#define 	MC_AUXOUT3						16
		#define		MC_AUXOUT4						17
		#define 	MC_ACTUAL_VELOCITY_SP			18
		#define		MC_ACTUAL_CURRENT_SP			19
		#define 	MC_REQUESTED_CURRENT_SP			20
		#define     MC_REQUESTED_VELOCITY_SP		21   
		#define		MC_REQUESTED_PWM_SP				22
		#define 	MC_NUM_OUT_CHANNELS				23
	/* In Channels */
		#define 	MC_CURRENT_SP					0
		#define 	MC_VELOCITY_SP					1      
		#define 	MC_PWM_SP						2
		#define		MC_NUM_IN_CHANNELS				3   
	/* Configuration Parameters */
		#define 	MC_INVERSE_SPEEDLOOP_P			0
		#define 	MC_INVERSE_SPEEDLOOP_I			1
		#define 	MC_INVERSE_SPEEDLOOP_D			2
		#define 	MC_WHEEL_DIAMETER				3

/* New Generation MPPT - Biel */
	/* Out Channels */
		#define MPPTNG_IN_VOLTAGE				0
		#define MPPTNG_IN_CURRENT				1
		#define MPPTNG_OUT_VOLTAGE				2
		#define MPPTNG_SENSE_VOLTAGE			3
		#define MPPTNG_HEATSINK_TEMP			4
		#define MPPTNG_AMBIENT_TEMP				5
		#define MPPTNG_TARGET_VOLTAGE			6
	/* In Channels - None (To be updated) */

/* Support Car */
        /* Out Channels */
                #define SUPPORTCAR_BR_BAT_FAN                   0
                #define SUPPORTCAR_FR_BAT_FAN                   1
                #define SUPPORTCAR_FL_BAT_FAN                   2
                #define SUPPORTCAR_BL_BAT_FAN                   3
                #define SUPPORTCAR_MESSAGE_OUT                  4
                #define SUPPORTCAR_SETSPEED_OUT                 5
		#define SUPPORTCAR_NUM_OUT_CHANNELS		6
	/* In Channels: None */
		#define SUPPORTCAR_NUM_IN_CHANNELS		0

/* Hydra */
	/* Fix Me */
		#define HYDRA_NUM_OUT_CHANNELS		0
	/* In Channels: None */
		#define HYDRA_NUM_IN_CHANNELS		0

/* GPS */
		#define GPS_TIME			0
		#define	GPS_LATITUDE			1
		#define	GPS_LONGITUDE			2
		#define GPS_ALTITUDE			3
		#define GPS_SPEED			4
		#define GPS_ODOMETER			5
		#define GPS_NUM_OUT_CHANNELS		6
	/* In Channels: None */
		#define GPS_NUM_IN_CHANNELS		0

/* One Wire Temp Sensor Node */
	/* Out Channels */
                #define TEMPSENSOR_REFNODE_CURRENT              0
                #define TEMPSENSOR_AMBIENT	                1
		#define TEMPSENSOR_SENSOR0			2
		#define TEMPSENSOR_SENSOR1			3
		#define TEMPSENSOR_SENSOR2			4
		#define TEMPSENSOR_SENSOR3			5
		#define TEMPSENSOR_SENSOR4			6
		#define TEMPSENSOR_SENSOR5			7
		#define TEMPSENSOR_SENSOR6			8
		#define TEMPSENSOR_SENSOR7			9
		#define TEMPSENSOR_NUM_OUT_CHANNELS		10
	/* In Channels: None */
		#define TEMPSENSOR_NUM_IN_CHANNELS		0

/* Negative sum node */
        /* Out channels */
                
                #define NEGATIVESUM_BUSVOLTAGE                  0
                #define NEGATIVESUM_CURRENT1                    1
                #define NEGATIVESUM_CURRENT2                    2
                #define NEGATIVESUM_CURRENT3                    3
                #define NEGATIVESUM_5V_V                        4
                #define NEGATIVESUM_12V_V                       5
                #define NEGATIVESUM_12V_I                       6
                #define NEGATIVESUM_AMBIENT_TEMP                7

/* Channels below here do not need scaling */ 
                #define NEGATIVESUM_POWER1                      8
                #define NEGATIVESUM_POWER2                      9
                #define NEGATIVESUM_POWER3                      10
                #define NEGATIVESUM_CURRENT1_INT                11
                #define NEGATIVESUM_CURRENT2_INT                12
                #define NEGATIVESUM_CURRENT3_INT                13
                
                #define NEGATIVESUM_NUM_OUT_CHANNELS            8

        /* In Channels */
                #define NEGATIVESUM_NUM_IN_CHANNELS             0 


        /* User configuration */
/*                #define NEGATIVESUM_SET_CURRENT1_INT            0
                #define NEGATIVESUM_SET_CURRENT2_INT            1
                #define NEGATIVESUM_SET_CURRENT3_INT            2
                #define NEGATIVESUM_SET_POWER1_INT              3
                #define NEGATIVESUM_SET_POWER2_INT              4
                #define NEGATIVESUM_SET_POWER3_INT              5
*/

/* UQM Interface */
	/* Out Channels */
                #define 	MC_ACTUALPWM					0
		#define 	MC_ACTUALI						1
		#define 	MC_ACTUALVEL					2
		#define 	MC_BUSV							3
		#define 	MC_CONTROLLERI					4
		#define 	MC_HEATSINKTEMP					5
		#define 	MC_MOTORTEMP					6
		#define 	MC_CONTROLLERTEMP				7
		#define 	MC_SMPSTEMP						8
		#define 	MC_15V							9
		#define 	MC_HORN							10
		#define 	MC_BRAKE						11
		#define 	MC_RIGHTINDICATOR				12
		#define 	MC_LEFTINDICATOR				13
		#define 	UQMINTERFACE_NUM_OUT_CHANNELS			14
	/* In Channels */
		#define 	MC_CURRENT_SP					0
		#define 	MC_VELOCITY_SP					1
		#define		UQMINTERFACE_NUM_IN_CHANNELS			2

/* 4-Port SwitchCard  */

	/* Out Channels */
		#define SWITCHCARD4_VOLTAGE_0				0
		#define SWITCHCARD4_VOLTAGE_1				1
		#define SWITCHCARD4_VOLTAGE_2				2
		#define SWITCHCARD4_VOLTAGE_3				3
		#define SWITCHCARD4_AMBIENT					4
		#define SWITCHCARD4_REFNODE_CURRENT			5
		#define SWITCHCARD4_CHANNEL_0_STATUS			6
		#define SWITCHCARD4_CHANNEL_1_STATUS			7
		#define SWITCHCARD4_CHANNEL_2_STATUS			8
		#define SWITCHCARD4_CHANNEL_3_STATUS			9
		#define SWITCHCARD4_NUM_OUT_CHANNELS			10

	/* In Channels */
		#define SWITCHCARD4_CHANNEL_0				0
		#define SWITCHCARD4_CHANNEL_1				1
		#define SWITCHCARD4_CHANNEL_2				2
		#define SWITCHCARD4_CHANNEL_3				3
		#define SWITCHCARD4_NUM_IN_CHANNELS			4


/* Data logger  */
	/* Out Channels */
                #define DATALOGGER_NUM_LOGGED_PACKETS                   0  
		#define DATALOGGER_NUM_OUT_CHANNELS			1

	/* In Channels */
		#define DATALOGGER_NUM_IN_CHANNELS			0

	/* Errors */ 
		#define DATALOGGER_FILE_NOT_FOUND			0

/* Courseprofile  */
	/* Out Channels */
                #define COURSEPROFILE_LATITUDE                          0  
                #define COURSEPROFILE_LONGITUDE                         1  
                #define COURSEPROFILE_HEIGHT                            2  
                #define COURSEPROFILE_CHAINAGE                          3  
                #define COURSEPROFILE_SLOPE                             4  
                #define COURSEPROFILE_ROUGHNESS                         5  
		#define COURSEPROFILE_NUM_OUT_CHANNELS			6

	/* In Channels */
		#define COURSEPROFILE_NUM_IN_CHANNELS			0

/* Mini-NG MPPT single node */
		/* This will go away */
		#define MININGSGL_TESTCHAN				0
		#define MININGSGL_PWM					1
		#define MININGSGL_NUM_OUT_CHANNELS			2

		/* Testing: allow a set PWM to be dialed in over CAN
		 * This should go away so that there isn't a way to 
		 * accidently make the tracker do bad things in normal
		 * operation
		 */
		#define MININGSGL_PWMIN					0
		#define MININGSGL_NUM_IN_CHANNELS			1

/* Mini-NG MPPT multi-module node */
 		#define MINING_NUM_OUT_CHANNELS				0

		#define MINING_NUM_IN_CHANNELS				0

/* Mini-NG MPPT multi-module node */
                #define MINING2_TIME                                    0
 		#define MINING2_NUM_OUT_CHANNELS			0

                #define MINING2_PWM_COMMAND                             0
		#define MINING2_NUM_IN_CHANNELS				0

/* Tilt sensor */
		#define TILT_X_AXIS					0
		#define TILT_Y_AXIS					1
		#define TILT_TEMP					2

                #define TILT_NUM_OUT_CHANNELS			        3
		#define TILT_NUM_IN_CHANNELS				0

/* MPPT New Generation, UNSW Control Board */
	/* Out Channels */
		#define UNSWMPPTNG_IN_VOLTAGE				0
		#define UNSWMPPTNG_IN_CURRENT				1
		#define UNSWMPPTNG_OUT_VOLTAGE				2
		#define UNSWMPPTNG_15V			                3
		#define UNSWMPPTNG_HEATSINK_TEMP			4
		#define UNSWMPPTNG_AMBIENT_TEMP				5
                #define UNSWMPPTNG_NUM_OUT_CHANNELS                     6

/* Unofficial out channels -- no scaling */ 
                #define UNSWMPPTNG_STATUS                               (UNSWMPPTNG_NUM_OUT_CHANNELS + 0)
                #define UNSWMPPTNG_SWEEP_IN_VOLTAGE                     (UNSWMPPTNG_NUM_OUT_CHANNELS + 1)
                #define UNSWMPPTNG_SWEEP_IN_CURRENT                     (UNSWMPPTNG_NUM_OUT_CHANNELS + 2)
                #define UNSWMPPTNG_PANDO_POWER                          (UNSWMPPTNG_NUM_OUT_CHANNELS + 3)

	/* In Channels - None (To be updated) */
                #define UNSWMPPTNG_NUM_IN_CHANNELS                      0

        /* Configuration parameters */ 
                #define UNSWMPPTNG_MAX_VOUT                             0
                #define UNSWMPPTNG_MIN_VIN                              1
                #define UNSWMPPTNG_ALGORITHM                            2
                #define UNSWMPPTNG_IN_KP                                3
                #define UNSWMPPTNG_IN_KI                                4
                #define UNSWMPPTNG_IN_KD                                5
                #define UNSWMPPTNG_OUT_KP                               6
                #define UNSWMPPTNG_OUT_KI                               7
                #define UNSWMPPTNG_OUT_KD                               8
                #define UNSWMPPTNG_OPENLOOP_RATIO                       9
                #define UNSWMPPTNG_OPENLOOP_RETRACK_PERIOD              10
                #define UNSWMPPTNG_PANDO_INCREMENT                      11
                #define UNSWMPPTNG_IVSWEEP_SAMPLE_PERIOD                12
                #define UNSWMPPTNG_IVSWEEP_STEP_SIZE                    13

        /* Errors */ 
                #define UNSWMPPTNG_ERROR_NONE                           0
                #define UNSWMPPTNG_ERROR_EEPROM                         1
                #define UNSWMPPTNG_ERROR_OUTPUT_OVER_VOLTAGE            2
                #define UNSWMPPTNG_ERROR_INPUT_UNDER_VOLTAGE            3
                #define UNSWMPPTNG_ERROR_FPGA_SHUTDOWN                  4
                #define UNSWMPPTNG_ERROR_BATTERY_FULL                   5
                #define UNSWMPPTNG_ERROR_WATCHDOG_RESET                 6


        /* Commands */ 
                #define UNSWMPPTNG_COMMAND_IVSWEEP                      0
                #define UNSWMPPTNG_COMMAND_SET_TARGET                   1
                #define UNSWMPPTNG_COMMAND_SET_AND_TUNE                 2
                #define UNSWMPPTNG_NUM_COMMANDS                         3

/* GPS Mouse interface */
	/* Out Channels */
                #define GPSMOUSE_TIME                                   0
                #define GPSMOUSE_LATITUDE                               1
                #define GPSMOUSE_LONGITUDE                              2
                #define GPSMOUSE_ALTITUDE                               3
                #define GPSMOUSE_SPEED                                  4
                #define GPSMOUSE_MILLISECONDS_TODAY                     5
                #define GPSMOUSE_DAYS_SINCE_EPOCH                       6
                #define GPSMOUSE_NUM_OUT_CHANNELS                       7

	/* In Channels */
                #define GPSMOUSE_NUM_IN_CHANNELS                        0



/* MSP Lcd */
        /* Out Channels */
		#define MSPLCD_NUM_OUT_CHANNELS				0

        /* In Channels */
		#define MSPLCD_CHANNEL_1					0
		#define MSPLCD_CHANNEL_2					1
		#define MSPLCD_CHANNEL_3					2
		#define MSPLCD_CHANNEL_4					3
		#define MSPLCD_CHANNEL_5					4
		#define MSPLCD_CHANNEL_6					5
		#define MSPLCD_CHANNEL_7					6
		#define MSPLCD_CHANNEL_8					7
		#define MSPLCD_CHANNEL_9					8
		#define MSPLCD_CHANNEL_10					9
		#define MSPLCD_MESSAGE						10
		#define MSPLCD_NUM_IN_CHANNELS				11


/* MSP Switch Card */
	/* Out Channels */
		#define MSPSWITCH_IN_VOLTAGE			0
		#define MSPSWITCH_VOLTAGE_1				1
		#define MSPSWITCH_VOLTAGE_2				2
		#define MSPSWITCH_VOLTAGE_3				3
		#define MSPSWITCH_VOLTAGE_4				4
		#define MSPSWITCH_CHANNEL_1_STATUS		5
		#define MSPSWITCH_CHANNEL_2_STATUS		6
		#define MSPSWITCH_CHANNEL_3_STATUS		7
		#define MSPSWITCH_CHANNEL_4_STATUS		8
		#define MSPSWITCH_AMBIENT				9
		#define MSPSWITCH_NUM_OUT_CHANNELS			10

	/* In Channels */
		#define MSPSWITCH_CHANNEL_1				0
		#define MSPSWITCH_CHANNEL_2				1
		#define MSPSWITCH_CHANNEL_3				2
		#define MSPSWITCH_CHANNEL_4				3
		#define MSPSWITCH_NUM_IN_CHANNELS			4

/*LHS Driver Controls*/
	/* Out Channels */
                #define LHSCONTROLS_LH_IND                              0
                #define LHSCONTROLS_RH_IND                              1
                #define LHSCONTROLS_HORN                                2
                #define LHSCONTROLS_REARVISION                          3
                #define LHSCONTROLS_DRIVERDISPLAY_A                     4
                #define LHSCONTROLS_PUSHTALK                            5
                #define LHSCONTROLS_MISC_CAN                            6
                #define LHSCONTROLS_MOTOR_CONTROLLER_IGNITION           7
                #define LHSCONTROLS_NUM_OUT_CHANNELS			8

	/* In Channels */
		#define LHSCONTROLS_NUM_IN_CHANNELS			0


/* Tyre Master interface */
	/* Out Channels */
		#define TYREMASTER_PRESSURE				0
		#define TYREMASTER_AIR_TEMP				1
		#define TYREMASTER_BATT_VOLTAGE				2
		#define TYREMASTER_DISC_TEMP				3
		#define TYREMASTER_SAMPLES				4
		#define TYREMASTER_NUM_OUT_CHANNELS				0		

	/* In Channels */
		#define TYREMASTER_NUM_IN_CHANNELS			0


/* SculptorBridge Interface */
	/* Out Channels */
                #define SCULPTORBRIDGE_BUSCURRENT                       0
                #define SCULPTORBRIDGE_BUSVOLTS                         1
                #define SCULPTORBRIDGE_VELOCITY                         2
                #define SCULPTORBRIDGE_CURRENTA                         3
                #define SCULPTORBRIDGE_CURRENTB                         4
                #define SCULPTORBRIDGE_15V                              5
                #define SCULPTORBRIDGE_1_65V                            6
                #define SCULPTORBRIDGE_2_5V                             7
                #define SCULPTORBRIDGE_1_2V                             8
                #define SCULPTORBRIDGE_FAN_SPEED                        9
                #define SCULPTORBRIDGE_FAN_DRIVE                        10
                #define SCULPTORBRIDGE_BRAKE                            11
                #define SCULPTORBRIDGE_TEMP_HS                          12
                #define SCULPTORBRIDGE_TEMP_MOTOR                       13
                #define SCULPTORBRIDGE_TEMP_CAPS                        14
                #define SCULPTORBRIDGE_SET_VELOCITY                     15
                #define SCULPTORBRIDGE_SET_CURRENT                      16
                #define SCULPTORBRIDGE_SET_BUSCURRENT                   17
                #define SCULPTORBRIDGE_LIMITS                           18
		#define SCULPTORBRIDGE_NUM_OUT_CHANNELS			19

	/* In Channels */
		#define SCULPTORBRIDGE_NUM_IN_CHANNELS			0



/* Current Integrator */ 
/* Out Channels */ 
#define CURRENTINT_CURRENT                                              0
#define CURRENTINT_VOLTAGE                                              1
#define CURRENTINT_POWER                                           	2
#define CURRENTINT_CURRENTINT						3
#define CURRENTINT_POWERINT						4
#define CURRENTINT_SAMPLES						5
#define CURRENTINT_NUM_OUT_CHANNELS                                     6

/* In Channels */ 
#define CURRENTINT_NUM_IN_CHANNELS                                      0
//<<<<<<< scandal_devices.h

/* MSPOnewire */ 
/* Out Channels */ 
//Dynamic...depending on number of sensors connected. Starts from 1. 
#define MSPONEWIRE_NUM_OUT_CHANNELS					10

/* In Channels */ 
#define MSPONEWIRE_RESET						0
#define MSPONEWIRE_NUM_IN_CHANNELS					1
//=======


/* Smart DC-DC */

	/* Out channels */
		#define SMARTDCDC_BATT_VOLTAGE				0
		#define SMARTDCDC_DCDC_IN_CURRENT			1
		#define SMARTDCDC_DCDC_OUT_CURRENT			2
		#define SMARTDCDC_5V_VOLTAGE				3
		#define SMARTDCDC_12V_VOLTAGE				4
		#define SMARTDCDC_CAN_5V_VOLTAGE			5
		#define SMARTDCDC_CAN_12V_VOLTAGE			6
		#define SMARTDCDC_HEATSINK_TEMP				7
		#define SMARTDCDC_MSP_TEMP				8
		#define SMARTDCDC_WAVESCULPTOR_VOLTAGE			9
		#define SMARTDCDC_VICOR_STATUS				10
		#define SMARTDCDC_NUM_OUT_CHANNELS			11
 
	/* Unscaled channels don't need to be explicitly declared */
		#define SMARTDCDC_RELAY_STATUS				11
		#define SMARTDCDC_CONTACTOR_STATUS			12
		#define SMARTDCDC_PRECHARGE_STATUS			13
		#define SMARTDCDC_CAN_STATUS 				14
		#define SMARTDCDC_AUX1_STATUS				15
		#define SMARTDCDC_AUX2_STATUS				16
		
	/* In channels */
		#define SMARTDCDC_AUX1_SW				0
		#define SMARTDCDC_AUX2_SW				1
		#define SMARTDCDC_IGNITION				2
		#define SMARTDCDC_CAN_POWER				3
		#define SMARTDCDC_VI_J00				4

	/* Debug function. Do not touch or wavesculptor will blow up. Literally. */
		#define SMARTDCDC_RELAY_SWITCH				5
		#define SMARTDCDC_CONTACTOR_SWITCH			6
		#define SMARTDCDC_NUM_IN_CHANNELS			7
 
	/* Configuration parameters */
		#define SMARTDCDC_PRECHARGE_CONTACT_VOLTAGE		0
		#define SMARTDCDC_PRECHARGE_MINIMUM_VOLTAGE		1
		#define SMARTDCDC_PRECHARGE_MINIMUM_TIME		2
		#define SMARTDCDC_PRECHARGE_MAXIMUM_TIME		3
 
	/* Errors */
		#define SMARTDCDC_ERROR_NONE				0
		#define SMARTDCDC_ERROR_EEPROM				1
		#define SMARTDCDC_ERROR_CIRCUIT_OVERHEAT		2
		#define SMARTDCDC_ERROR_HEATSINK_OVERHEAT		3
		#define SMARTDCDC_ERROR_INPUT_UNDER_VOLTAGE		4
		#define SMARTDCDC_ERROR_WAVESCULPTOR_OVER_VOLTAGE	5
		#define SMARTDCDC_ERROR_ADC_SIGNAL_LOST_OH_SHIT		6
		#define SMARTDCDC_ERROR_PRECHARGE_OVERTIME		7
		#define SMARTDCDC_ERROR_RELAY				8
		#define SMARTDCDC_ERROR_CONTACTOR_FUSE_BLOWS_OH_FUCK 	9
		#define SMARTDCDC_ERROR_AUX1_FUSE_BLOWS			10
		#define SMARTDCDC_ERROR_AUX2_FUSE_BLOWS			11

/* SteeringWheel Interface */
	/* Out Channels */
		#define STEERINGWHEEL_VIN				0
		#define STEERINGWHEEL_THROTTLE				1
		#define STEERINGWHEEL_REGEN				2
		#define STEERINGWHEEL_AMBIENT_TEMP			3
		#define STEERINGWHEEL_NUM_OUT_CHANNELS			4
		
	/* Unofficial out channels -- no scaling */		
	
                #define STEERINGWHEEL_BUSCURRENT                       (STEERINGWHEEL_NUM_OUT_CHANNELS + 0)
                #define STEERINGWHEEL_BUSVOLTS                         (STEERINGWHEEL_NUM_OUT_CHANNELS + 1)
                #define STEERINGWHEEL_VELOCITY                         (STEERINGWHEEL_NUM_OUT_CHANNELS + 2)
                #define STEERINGWHEEL_CURRENTA                         (STEERINGWHEEL_NUM_OUT_CHANNELS + 3)
                #define STEERINGWHEEL_CURRENTB                         (STEERINGWHEEL_NUM_OUT_CHANNELS + 4)
                #define STEERINGWHEEL_15V                              (STEERINGWHEEL_NUM_OUT_CHANNELS + 5)
                #define STEERINGWHEEL_FAN_SPEED                        (STEERINGWHEEL_NUM_OUT_CHANNELS + 6)
                #define STEERINGWHEEL_BRAKE                            (STEERINGWHEEL_NUM_OUT_CHANNELS + 7)
                #define STEERINGWHEEL_TEMP_HS                          (STEERINGWHEEL_NUM_OUT_CHANNELS + 8)
                #define STEERINGWHEEL_TEMP_MOTOR                       (STEERINGWHEEL_NUM_OUT_CHANNELS + 9)
                #define STEERINGWHEEL_SET_VELOCITY                     (STEERINGWHEEL_NUM_OUT_CHANNELS + 10)
                #define STEERINGWHEEL_SET_CURRENT                      (STEERINGWHEEL_NUM_OUT_CHANNELS + 11)
                #define STEERINGWHEEL_SET_BUSCURRENT                   (STEERINGWHEEL_NUM_OUT_CHANNELS + 12)
                #define STEERINGWHEEL_LIMITS                           (STEERINGWHEEL_NUM_OUT_CHANNELS + 13)
                #define STEERINGWHEEL_ERRORS				(STEERINGWHEEL_NUM_OUT_CHANNELS + 14)
                #define STEERINGWHEEL_AMP_HOURS				(STEERINGWHEEL_NUM_OUT_CHANNELS + 15)
                #define STEERINGWHEEL_ODO				(STEERINGWHEEL_NUM_OUT_CHANNELS + 16)
                #define STEERINGWHEEL_LH_IND				(STEERINGWHEEL_NUM_OUT_CHANNELS + 19)
                #define STEERINGWHEEL_RH_IND				(STEERINGWHEEL_NUM_OUT_CHANNELS + 20)
                #define STEERINGWHEEL_HORN				(STEERINGWHEEL_NUM_OUT_CHANNELS + 21)
                #define STEERINGWHEEL_RADIO				(STEERINGWHEEL_NUM_OUT_CHANNELS + 22)
                #define STEERINGWHEEL_REAR_VISION			(STEERINGWHEEL_NUM_OUT_CHANNELS + 23)
                #define STEERINGWHEEL_START				(STEERINGWHEEL_NUM_OUT_CHANNELS + 24)

		
	
	/* In Channels */
		#define STEERINGWHEEL_LCD_1				0
		#define STEERINGWHEEL_LCD_2				1
		#define STEERINGWHEEL_LCD_3				2
		#define STEERINGWHEEL_LCD_4				3
		#define STEERINGWHEEL_LCD_5				4
		#define STEERINGWHEEL_LCD_6				5
		#define STEERINGWHEEL_LCD_7				6
		#define STEERINGWHEEL_LCD_8				7
		#define STEERINGWHEEL_LCD_MSG				8
		#define STEERINGWHEEL_PRECHARGE_STAT			9
		#define STEERINGWHEEL_NUM_IN_CHANNELS			10
	
		



/* GPS Barometer interface */
	/* Out Channels */
                #define GPSBAROMETER_TIME                                   0
                #define GPSBAROMETER_LATITUDE                               1
                #define GPSBAROMETER_LONGITUDE                              2
                #define GPSBAROMETER_ALTITUDE                               3
                #define GPSBAROMETER_SPEED                                  4
                #define GPSBAROMETER_MILLISECONDS_TODAY                     5
                #define GPSBAROMETER_DAYS_SINCE_EPOCH                       6
                #define GPSBAROMETER_TEMP                                   7
                #define GPSBAROMETER_PRESSURE                               8
                #define GPSBAROMETER_PRESSURE_8                             9
                #define GPSBAROMETER_PRESSURE_16                            10
                #define GPSBAROMETER_B_OPERATION_STATUS                     11 
                #define GPSBAROMETER_B_STATUS                               12
                #define GPSBAROMETER_NUM_OUT_CHANNELS                       13

	/* In Channels */
                #define GPSBAROMETER_NUM_IN_CHANNELS                        0


/* TEMPLATE interface */
	/* Out Channels */
                #define TEMPLATE_TEST_OUT                                   0
                #define TEMPLATE_NUM_OUT_CHANNELS                           1

	/* In Channels */
                #define TEMPLATE_TEST_IN                                    0
                #define TEMPLATE_NUM_IN_CHANNELS                            1


