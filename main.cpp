/*IDLE_ST E DEBUG_ST obrigatorios
* REAR: TEMP_MOTOR_ST, FUEL_ST, RPM_ST, THROTTLE_ST, RADIO_ST
* FRONT: SLOWACQ_ST, IMU_ST, SPEED_ST, THROTTLE_ST, DISPLAY_ST
* BMU: Voltage_ST, TEMP_CVT_ST, SystemCurrent_ST
*/

/*Novos:
* REAR: TEMP_MOTOR_ST, FUEL_ST, TEMP_CVT_ST, RPM_ST, RADIO_ST, THROTTLE_ST
*FRONT: SystemCurrent_ST, THROTTLE_ST, Voltage_ST, IMU_ST, SPEED_ST, DISPLAY_ST
*/
#include "mbed.h"
#include "stats_report.h"
/* User libraries */
#include "defs.h"
#include "rear_defs.h"
#include "CANMsg.h"
#include "MLX90614.h"
#include "RFM69.h"

#define default_addr   (0x00)

//#define MB1                   // uncomment this line if MB1
#define MB2                 // uncomment this line if MB2

#ifdef MB1
#define NODE_ID MB1_ID
#endif
#ifdef MB2
#define NODE_ID MB2_ID
#endif

/* Communication protocols */
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_2, PA_3, 115200);
RFM69 radio(PB_15, PB_14, PB_13, PB_12, PA_8); 
//RFM69::RFM69(PinName  PinName mosi, PinName miso, PinName sclk,slaveSelectPin, PinName int)
I2C i2c(PB_7, PB_6);
//sda,scl
MLX90614 mlx(&i2c);

/* I/O pins */
AnalogIn analog(PA_0);
AnalogIn fuel_read(PB_1);
PwmOut servo(PA_6);
/* Debug pins */
DigitalOut led(PC_13);
DigitalOut dbg1(PB_11);

/* Debug variables */
Timer t;
bool buffer_full = false;
uint32_t tim1, tim2, imu_last_acq = 0;

/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
CircularBuffer <state_t, 2*BUFFER_SIZE> state_buffer;
CircularBuffer <imu_t*, 20> imu_buffer;
Ticker ticker1Hz;
Ticker ticker5Hz;
Ticker ticker66mHz;

/* Global variables */
state_t current_state = IDLE_ST;
packet_t data;                    // Create package for radio comunication
bool switch_clicked = false;
float V_termistor=0, rpm_hz;
uint8_t switch_state = 0x00;
uint8_t MeasureCVTtemperature;

/* Interrupt handlers */
void canHandler();
/* Interrupt services routine */
void canISR();
void ticker1HzISR();
void ticker5HzISR();
void ticker66mHzISR();
/* General functions*/
void setupInterrupts();
void filterMessage(CANMsg msg);
float moving_averages();
uint8_t mode();
void initRadio();
double CVT_Temperature();
void writeServo(uint8_t state);

int main ()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    initRadio();
    setupInterrupts();

    while (true)
    {
        if (state_buffer.full()) 
        {
            buffer_full = true;
            led = 0;
        } else {
            led = 1;
            buffer_full = false;
            if (!state_buffer.empty())
            {
                state_buffer.pop(current_state);
            } else {
                current_state = IDLE_ST;
            }
        }

        switch (current_state) 
        {
            case IDLE_ST:
                //Thread::wait(1);
                break;

            case TEMP_MOTOR_ST:
                //serial.printf("t");
                V_termistor = VCC*analog.read();

                data.tempMOTOR = ((float) 115.5*(exp(-0.02187*(V_termistor*R_TERM)/(VCC - V_termistor))) + 85.97*(exp(-0.00146*(V_termistor*R_TERM)/(VCC - V_termistor))));

                /* Send temperature data */
                txMsg.clear(TEMPERATURE_ID);
                txMsg << data.tempMOTOR;
                can.write(txMsg);
                break;

            case FUEL_ST:
                data.fuel=mode();

                /*Can communication*/
                txMsg.clear(FUEL_ID);
                txMsg << data.fuel;
                can.write(txMsg);
                break;

            case TEMP_CVT_ST:
                MeasureCVTtemperature = (uint8_t)CVT_Temperature(); 

                //MeasureCVTtemperature = 90;
                /*Send temperature data*/
                txMsg.clear(TempCVT_ID);
                txMsg << MeasureCVTtemperature;
                can.write(txMsg); 

                break;

            case RPM_ST:
                rpm_hz=1;
                break;

            case RADIO_ST:
                imu_t* temp_imu;

                if (!imu_buffer.empty()) {
                        imu_buffer.pop(temp_imu);
                        memcpy(&data.imu, temp_imu, 4*sizeof(imu_t));
                        data.rpm = ((uint16_t)rpm_hz * 60)*65536.0/5000.0;
                        #ifdef MB1
                            radio.send((uint8_t)BOXRADIO_ID1, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                        #endif
                        #ifdef MB2
                            radio.send((uint8_t)BOXRADIO_ID2, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                            dbg1 = !dbg1;
                        #endif
                    } else if (t.read_ms() - imu_last_acq > 500) {
                        memset(&data.imu, 0, 4*sizeof(imu_t));
                        data.rpm = ((uint16_t)rpm_hz * 60)*65536.0/5000.0;
                        #ifdef MB1
                            radio.send((uint8_t)BOXRADIO_ID1, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                        #endif
                        #ifdef MB2
                            radio.send((uint8_t)BOXRADIO_ID2, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                        #endif
                    }
                break;

            case THROTTLE_ST:
                if (switch_clicked) {
                    if (rpm_hz != 0) {
                        writeServo(CHOKE_MODE);
                    }
                    else {
                        writeServo(switch_state);
                    }
                    switch_clicked = false;
                }

                break;

            case DEBUG_ST:
                break;
        }
    }
}

void setupInterrupts()
{
    can.attach(&canISR, CAN::RxIrq);
    ticker1Hz.attach(&ticker1HzISR, 1.0);
    ticker5Hz.attach(&ticker5HzISR, 0.2);
    ticker66mHz.attach(&ticker66mHzISR, 4);
}

/* Interrupt handlers */
void canHandler()
{
    CANMsg rxMsg;

    can.read(rxMsg);
    filterMessage(rxMsg);
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}

/* Interrupt services routine */
void canISR()
{
    CAN_IER &= ~CAN_IER_FMPIE0;                 // disable RX interrupt
    queue.call(&canHandler);                    // add canHandler() to events queue
}

void ticker1HzISR()
{
    state_buffer.push(TEMP_MOTOR_ST);
    state_buffer.push(FUEL_ST);
}

void ticker5HzISR()
{
    state_buffer.push(RPM_ST);
    state_buffer.push(RADIO_ST);
}

void ticker66mHzISR()
{
    state_buffer.push(TEMP_CVT_ST);
}

/* General functions*/
void filterMessage(CANMsg msg)
{
    led = !led;
    //serial.printf("alalala1");

    if (msg.id == THROTTLE_ID) {
        switch_clicked = true;
        state_buffer.push(THROTTLE_ST);
        msg >> switch_state;
    } else if (msg.id == IMU_ACC_ID) {
        imu_last_acq = t.read_ms();
        msg >> data.imu.acc_x >> data.imu.acc_y >> data.imu.acc_z;
        //msg >> data.imu.acc_x >> data.imu.acc_y >> data.imu.acc_z;
    } else if (msg.id == IMU_DPS_ID) {
        msg >> data.imu.dps_x >> data.imu.dps_y >> data.imu.dps_z;
        //msg >> data.imu.dps_x >> data.imu.dps_y >> data.imu.dps_z;
        /*if (imu_counter < 3) {
            imu_counter++;
        } else if (imu_counter == 3) {
            imu_buffer.push(data.imu);
            imu_counter = 0;
        }*/
    } else if (msg.id == SPEED_ID) {
        msg >> data.speed;
        //serial.printf("alalala2");
        //serial.printf("\r\nspeed = %d\r\n",data.data_10hz[packet_counter[N_SPEED]].speed);
        //d10hz_buffer.push(data.data_10hz);
    } else if (msg.id == Voltage_ID) {
        msg >> data.voltage;
    } else if (msg.id == SOC_ID) {
        msg >> data.soc;
    }
}

uint8_t mode() 
{
    const int T=20;
    uint8_t cont[T],current_level;
    float R_final_average[T];
    float moda, R_final, R_final_mode;
    float count = 15.00; // o valor deve aparecer 15 vezes em 20 para ser lido

    for (int i=1; i<T; i++) { 
    // mover enquanto acrescenta ao vetor (shift left)
      R_final = moving_averages();
      R_final_average[0] = R_final;
      R_final_average[i] = R_final_average[i-1];
    }

    for(int i=0; i<T; i++) {                 
        for(int j=i+1; j<T; j++) {
            if(R_final_average[i]==R_final_average[j]) {  // compara os valores
                cont[i]++;
                if(cont[i] > count) {  // se o valor aparecer em 15 das 20 vezes
                moda = R_final_average[i];
                }
            } 
        }
    cont[i] = 0; // reseta a lista
    }
    R_final_mode = moda;

    // respostas possíveis

  if (R_final_mode >= 175) { 
    // Acima de 60% no tanque (início da medição), 5.68L
      current_level = 100.00;
    } 
  else if (115 <= R_final_mode && R_final_mode < 175) {
    // Cerca de 50% do tanque, 2.84L
        current_level = 50.00;
    }
  else if (75 <= R_final_mode && R_final_mode < 115) { 
    // Cerca de 35% do tanque, 2L
        current_level = 35.00;
    } 
  else if (35 <= R_final_mode && R_final_mode < 75) {
    // Cerca de 25% do tanque, 1.42L
        current_level = 25.00;
    }
  else { 
    // Alcance da reserva, cerca de 18% do tanque, 1L
        current_level = 18.00;
    }

    return current_level;
}

float moving_averages() {
    // 2 filtros no formato de médias móveis
    uint8_t n=30;
    int R_average[n]; // vetor
    uint16_t read=0;
    float V, Rx, Rx_acc, R, R_acc, r_final=0.0;

    for (int i=0; i<(FUELsample); i++) {
      read = fuel_read.read_u16();       // Saída no Serial
      V = (read * ADCVoltageLimit) / 65535.0; // V = Voltagem da Boia
      Rx = (120 * V) / (3.3 - V); // Rx = Resistência da Boia
      Rx_acc += Rx;               // Rx_acc = acumulador do filtro 1
    }
      R = Rx_acc / (FUELsample); // faz a média
      Rx_acc = 0.0; // reseta o acc 
      //thread_sleep_for(200);

    for (int i=1; i<n; i++) { // mover pelo vetor (shift left)
      R_average[0] = R;
      R_average[i] = R_average[i-1];
    }
    
    for (int j=0; j<n; j++) { // soma os valores no acumulador
      R_acc += R_average[j]; 
    }
    r_final = R_acc / 30;  // faz a média
    R_acc = 0.0;   // reseta o acumulador 
    
    return r_final;
}

void initRadio()
{
    radio.initialize(FREQUENCY_915MHZ, NODE_ID, NETWORK_ID);
    radio.encrypt(0);
    radio.setPowerLevel(31);
    radio.setHighPower();
}

double CVT_Temperature()
{
    int i,j;
    double AverageObjectTemp, AverageEnviromentTemp = 0.0;
    double temp_amb,med_amb,x_amb;
    double temp_obj,med_obj,x_obj; 

    //teste comunicação i2c
    char ucdata_write[2];

 if (!i2c.write((default_addr|0x00), ucdata_write, 1, 0))
 {      // Check for ACK from i2c Device

    for(j = 0; j < (CVTsample); j++){ 

        for(i = 0; i < (CVTsample) ; i++){
                      
            // temp_amb = mlx.read_temp(0);
           

                temp_obj = mlx.read_temp(1);
                x_obj += temp_obj;
                med_obj = x_obj/ (double)CVTsample;
    
                if(med_obj > AverageObjectTemp){
                AverageObjectTemp = med_obj;
                 }
            }
        }
    } else {
        AverageObjectTemp = 0;
    }

    //return value;
    return AverageObjectTemp/(double)CVTsample; // I don't know why we need divide by sample, but works.
}

void writeServo(uint8_t state)
{
    data.flags &= ~(0x07);         // reset servo-related flags

    switch (state) {
        case MID_MODE:
            //dbg3 = !dbg3;
            servo.pulsewidth_us(SERVO_MID);
            data.flags &= ~(0x03); // reset run and choke flags
            break;
        case RUN_MODE:
            //dbg3 = !dbg3;
            servo.pulsewidth_us(SERVO_RUN);
            data.flags |= RUN_MODE;    // set run flag
            break;
        case CHOKE_MODE:
            //dbg3 = !dbg3;
            servo.pulsewidth_us(SERVO_CHOKE);
            data.flags |= CHOKE_MODE;    // set choke flag
            break;
        default:
            //serial.printf("Choke/run error\r\n");
            data.flags |= 0x04;    // set servo error flag
            break;
    }
}