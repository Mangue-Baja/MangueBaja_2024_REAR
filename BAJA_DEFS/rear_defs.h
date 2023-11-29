#ifndef REAR_DEFS_
#define REAR_DEFS_

/* Moving Average Definitions */
//#define VCC                 3.3
#define ADCVoltageLimit     3.3
#define SensorADClimit      3.2
#define R_TERM              1000
#define CVTsample           50
#define LevelSample         50
#define DENSITY             1.3565
#define sample              150 

/* Wheel Definitions */
#define PI                  3.1416
#define WHEEL_DIAMETER      0.5842      // m
//#define WHEEL_HOLES_NUMBER_MB1  24
#define WHEEL_HOLES_NUMBER_REAR   12
#define WHEEL_HOLES_NUMBER_FRONT  24

/* Servo definitions */
#define MID_MODE            0x00
#define RUN_MODE            0x01
#define CHOKE_MODE          0x02
#define SERVO_MID           1180
#define SERVO_RUN           880
#define SERVO_CHOKE         1480 //1000 -> 1200

typedef enum
{
    IDLE_ST,            // wait
    TEMP_MOTOR_ST,      // measure temperature of motor
    TEMP_CVT_ST,        // measure temperature of CVT
    FUEL_ST,            // proccess fuel data sampling
    SPEED_ST,           // calculate speed
    VOLTAGE_ST,         // calculate State of Charge and battery voltage
    SYSTEM_CURRENT_ST,  // measure the current of the system
    THROTTLE_ST,        // write throttle position (PWM)
    DEBUG_ST            // send data for debug

} state_t;

#endif