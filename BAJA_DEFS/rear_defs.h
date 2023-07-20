#ifndef REAR_DEFS_
#define REAR_DEFS_


#define VCC                 3.3
#define R_TERM              1000
#define ADCVoltageLimit     3.3 
#define CVTsample           50
#define FUELsample          200
#define CHOKE_MODE          0x02
#define SERVO_RUN           880
#define RUN_MODE            0x01
#define SERVO_MID           1180
#define MID_MODE            0x00
#define SERVO_CHOKE         1480 //1000 -> 1200

/* Radio definitions */
#define NETWORK_ID          101
#define BOXRADIO_ID1         69
#define BOXRADIO_ID2         70
#define MB1_ID              11
#define MB2_ID              22
#define FREQUENCY_915MHZ    91
#define NORMAL_THRESHOLD    68

//REAR: TEMP_MOTOR_ST, FUEL_ST, TEMP_CVT_ST, RPM_ST, RADIO_ST, THROTTLE_ST
typedef enum
{
    IDLE_ST,        // wait
    TEMP_MOTOR_ST,  // measure temperature of motor
    FUEL_ST,        // proccess fuel data sampling
    TEMP_CVT_ST,    // measure temperature of CVT
    RPM_ST,         // calculate speed
    RADIO_ST,       // send data for box via radio (SPI)
    THROTTLE_ST,    // write throttle position (PWM)
    DEBUG_ST        // send data for debug

} state_t;

#endif