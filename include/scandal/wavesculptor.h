#include <scandal/tritium.h>
#include <scandal/can.h>
#include <scandal/engine.h>


/* Multiplier constant to convert rpm to km/hr assuming 27 cm tyre radius
   This should be fixed for each car assuming a constant tyre diameter and no gearing
   if these assumptions are invalidated at some point in the future, you'll need to
   figure out how to do that :)
   RPM_TO_KMH_MULTIPLIER = (2*Pi*radius*RPM)*(60/1000)
   Set RPM to 1 for the multiplier
*/
#define RPM_TO_KMH_MULTIPLIER 0.101787602
#define KMH_TO_RPM_MULTIPLIER 9.824379203

// Motor controller CAN base address and packet offsets
#define MC_IDENTITY     0x00        // High = Serial Number             Low = Tritium ID (W20 = TRIa, WS22 = T088)
#define MC_LIMITS		0x01		// High = Active Motor/CAN counts   Low = Error & Limit flags
#define	MC_BUS			0x02		// High = Bus Current               Low = Bus Voltage
#define MC_VELOCITY		0x03		// High = Velocity (m/s)            Low = Velocity (rpm)
#define MC_PHASE		0x04		// High = Phase A Current           Low = Phase B Current
#define MC_V_VECTOR		0x05		// High = Vd vector                 Low = Vq vector
#define MC_I_VECTOR		0x06		// High = Id vector                 Low = Iq vector
#define MC_BEMF_VECTOR	0x07		// High = BEMFd vector              Low = BEMFq vector
#define MC_RAIL1		0x08		// High = 15V                       Low = Unused
#define MC_RAIL2		0x09		// High = 3.3V                      Low = 1.9V
#define MC_FAN			0x0A		// High = Reserved                  Low = Reserved
#define MC_TEMP1		0x0B		// High = Heatsink Phase C Temp     Low = Motor Temp
#define MC_TEMP2		0x0C		// High = Heatsink Phase B Temp     Low = CPU Temp
#define MC_TEMP3		0x0D		// High = Heatsink Phase A Temp     Low = Unused
#define MC_CUMULATIVE	0x0E		// High = DC Bus AmpHours           Low = Odometer

#define MC_MAXOFFSET    0x1F        // Maximum possible offset permitted from a given base address

#define DC_DRIVE_OFFSET 0x01        // High = Phase current             Low = Motor velocity (
#define DC_POWER_OFFSET 0x02        //

#define WS_20           20
#define WS_22           22



typedef struct _Wavesculptor_Output_Struct{
    uint16_t    BaseAddress;            // Tritium base address for sending messages 
    uint16_t    DriveAddress;           // Tritium base address from which control messages are accepted
    uint16_t    ControlAddress;         // Driver controls address for receiving drive messages
    sc_time_t   lastUpdateTime;         // Last update time of fields, if this is old - our data is stale
    uint32_t    SerialNumber;           // Serial number on both WS20 and WS22 models
    char        TritiumID[4];           // Model identifier string "" on WS20 and "" on WS22
    uint8_t     RXErrorCount;           // Reserved on WS20, CAN RX Error Count on WS22
    uint8_t     TXErrorCount;           // Reserved on WS20, CAN TX Error Count on WS22
    uint16_t    ActiveMotor;            // Active motor model
    uint16_t    ErrorFlags;             // Error flags
    uint16_t    LimitFlags;             // Limit flags
    float       BusCurrent;             // Bus current (A)
    float       BusVoltage;             // Bus voltage (V)
    float       VehicleVelocity;        // Vehicle velocity (m/s)
    float       MotorVelocity;          // Motor angular velocity (rpm)
    float       Phase_1;                // WS20: Phase A Current, WS22: Phase C Current
    float       Phase_2;                // WS20: Phase B Current, WS22: Phase B Current
    float       MotorVoltageVec_Re;     // Motor voltage vector (real part)
    float       MotorVoltageVec_Im;     // Motor voltage vector (imaginary part)
    float       MotorCurrentVec_Re;     // Motor current vector (real part)
    float       MotorCurrentVec_Im;     // Motor current vector (imaginary part)
    float       MotorBEMF_Re;           // Motor Back EMF model (real part)
    float       MotorBEMF_Im;           // Motor Back EMF model (imaginary part)
    float       VoltageRail_1;          // Internal 15V rail
    float       VoltageRail_2;          // WS20: 1.65V rail, WS22: reserved
    float       VoltageRail_3;          // WS20: 2.5V rail, WS22: 3.3V rail
    float       VoltageRail_4;          // WS20: 1.2V rail, WS22: 1.9V rail
    float       FanSpeed;               // WS20: Fan speed (rpm), WS22: reserved
    float       FanDrive;               // WS20: Fan Drive (%), WS22: reserved
    float       HeatsinkTemp;           // Heatsink temperature (Deg C)
    float       MotorTemp;              // Motor temperature (Deg C)
    float       AirInletTemp;           // WS20: Air inlet temperature (Deg C), WS22: reserved
    float       ProcessorTemp;          // Processor temperature (Deg C)
    float       AirOutletTemp;          // WS20: Outlet air temperature (Deg C), WS22: reserved
    float       CapacitorTemp;          // WS20: Capacitor bank temperature (Deg C), WS22: reserved
    float       DCBusAHr;               // Cumilative charge drawn from the bus since reset (AHr)
    float       Odometer;               // Cumilative distance travelled by vehicle since reset (m)
} Wavesculptor_Output_Struct;

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
uint8_t scandal_store_ws_message(can_msg *msg, Wavesculptor_Output_Struct *dataStruct);
int32_t check_device_type(Wavesculptor_Output_Struct *dataStruct);
void send_ws_drive_commands(float rpm, float phase_current, float bus_current, Wavesculptor_Output_Struct *dataStruct);
