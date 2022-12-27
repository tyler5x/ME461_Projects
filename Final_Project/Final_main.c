//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);
__interrupt void ADCD_ISR (void); // Distance Sensor ADC ISR

void setupSpib(void);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float);
void setEPWM2B(float);

extern int16_t ESP8266whichcommand;
extern int16_t ESP8266insidecommands;

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint32_t count = 0;
int16_t toAdd = 10;
uint32_t numADCD1InterruptCalls = 0;

int16_t rawD0 = 0;
int16_t rawD1 = 0;
float voltD0 = 0.0;
float voltD1 = 0.0;

float spi1 = 0.0;
float spi2 = 0.0;
float spi3 = 0.0;

int16_t raw_accelx = 0;
int16_t raw_accely = 0;
int16_t raw_accelz = 0;
int16_t raw_temp = 0;
int16_t raw_gyrox = 0;
int16_t raw_gyroy = 0;
int16_t raw_gyroz = 0;

float accelx = 0.0;
float accely = 0.0;
float accelz = 0.0;
float gyrox = 0.0;
float gyroy = 0.0;
float gyroz = 0.0;

float LeftWheelL6 = 0.0;
float RightWheelL6 = 0.0;
float LeftTravel = 0.0;
float RightTravel = 0.0;
float uLeft = 2.0;
float uRight = 7.0;

float WR = 0.56759;
float RWh = 0.1961;
float theta_r = 0.0;
float theta_l = 0.0;
float bearing = 0.0;
float bearing_mod = 0.0; //Absolute angle
float theta_avg = 0.0;
float t_dot_r = 0.0;
float t_dot_l = 0.0;
float t_dot_avg = 0.0; // Need to defined t_dots still
float x_dot_R = 0.0;
float y_dot_R = 0.0;
float xRK = 0.0;
float yRK = 0.0;

float PosLeft_K = 0.0; //Current position of left wheel
float PosLeft_K_1 = 0.0; //Previous position of left wheel
float PosRight_K = 0.0; //Current position of right wheel
float PosRight_K_1 = 0.0; //Previous position of right wheel
float VLeftK = 0.0;
float VRightK = 0.0;

float eLeftK = 0.0;
float eRightK = 0.0;
float eLeftK_1 = 0.0;
float eRightK_1 = 0.0;
float eturn = 0.0;
float turn = 0.0;

float ILeftK = 0.0;
float IRightK = 0.0;
float ILeftK_1 = 0.0;
float IRightK_1 = 0.0;
float Kturn = 3.0;


float theta_r_1 = 0.0;
float theta_l_1 = 0.0;
float xRK_1 = 0.0;
float yRK_1 = 0.0;
float x_dot_R_1 = 0.0;
float y_dot_R_1 = 0.0;
// Lab 7 Exercise 3 Vars
//float PosLeft_K = 0.0; //Current position of left wheel
//float PosLeft_K_1 = 0.0; //Previous position of left wheel
//float PosRight_K = 0.0; //Current position of right wheel
//float PosRight_K_1 = 0.0; //Previous position of right wheel

// Exercise 3 vars
float thdLK = 0.0;
float thdRK = 0.0;
float thdLK_1 = 0.0;
float thdRK_1 = 0.0;
float LeftWheelK_1 = 0.0;
float RightWheelK_1 = 0.0;

float gyrorate_dot = 0.0;
float gyrorate_dotK_1 = 0.0;
float gyro_valueK_1 = 0.0;

float ubal = 0.0;

// Exercise 3 constants
float K1 = -60.0;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;


//Target velocities
float Vref = 0.0;

float uLeftK = 0.0;
float uRightK = 0.0;

//:Lab 7 Exercise 2 Given Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -0.55;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheel = 0; //Left wheel angle in radians probably
float RightWheel = 0;
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

//Lab 7 Exercise 1
uint32_t SPIB_int_count = 0;

// Lab 7 Exercise 4
float WhlDiff = 0.0;
float WhlDiffK_1 = 0.0;
float vel_WhlDiff = 0.0;
float vel_WhlDiffK_1 = 0.0;
float turnref = 0.0;
float errorDiff = 0.0;
float intDiff = 0.0;
float errorDiffK_1 = 0.0;
float intDiffK_1 = 0.0;
float turnrate = 0.0;
float turnrateK_1 = 0.0;
float turnrefK_1 = 0.0;
uint16_t line_turn = 0;
uint16_t is_line = 0;

float Kp = 1.0;
float Ki = 5.0;
float Kd = 0.05;
float FwdBackOffset = 0.0;

float errorSpeed = 0.0;
float errorSpeedK_1 = 0.0;
float intSpeed = 0.0;
float intSpeedK_1 = 0.0;
float sKp = 0.35;
float sKi = 1.5;
float speeddes = 0.0;
uint16_t time_inc = 0;
uint16_t stateVar = 20;
uint16_t min_dist = 2500;
uint16_t max_ir = 0;
uint16_t stop_dis = 2000;
uint16_t dis_count = 0;
float s0_avg = 0.0;
float s1_avg = 0.0;
float s2_avg = 0.0;
float line_turn_f = 0.0;
uint16_t obstacle = 0;

// Global Variables for
int16_t adcd0result = 0; // Sensor 1 Value
int16_t adcd1result = 0; // Sensor 2 Value
int16_t adcd2result = 0; // Sensor 3 Value


// Observer Constants
float Bz[6][2] ={{0.0, 0.0},
                 {33.9744426687984,0.0},
                 {1760481.43793715,0.0},
                 {0.0, 1760447.46349448},
                 {0.0, 0.0},
                 {0.0, 0.0}};

float L[6][4] = {{3.41674981274991,   0.208381551085633,   -1.50145120251551e-05,   1.36322697351031e-09},
                 {338.111244614756,    4.55524602229178,    -0.000653819805851787,   -1.91695085155306e-08},
                 {3539.60581623829,    1592.53343498137,    26.0280033213747,    0.000645379448322896},
                 {1.99913362652660,    0.974947425902729,   0.00261031691736474, 24.0000008436070},
                 {-0.125988310107894,  -0.0626990072172536, -0.000257593594745354,   -6.42594804475704e-05},
                 {-0.125900297552992,  -0.0626558927108524, -0.000257426540000878,   6.41765737307670e-05}};
float E[6][6] = {
                 {3.41674981274991, 1.20838155108563,-.0000150,0.0,0.0,0.0},
                 {677.112481241045, 4.55524602229178, -0.000653819805851787,0.0, 5.26446507793355, 5.26446507793355},
                 {3878.60705286458, 1592.53343498137, 26.0280033213747, 0.000645379448322896, -171286.273732935, -171286.273732935},
                 {-0.125988310107894,-0.0626990072172536,-0.000257593594745354,-6.42594804475704e-05,0.0,0.0},
                 {-0.125900297552992,-0.0626558927108524,-0.000257426540000878, 6.41765737307670e-05,0.0,0.0}};

// Matrices for calculating ideal traction
float M2[2][2] = {{5.13825967604405, 2.56914677350707},{5.13825967604405,-2.56914677350707}};
float M1[2][4] = {{-5.17432347412347e-09,1.21649507849576e-07,-1.75923721349880e-07,-204.981370868571},{0.000136590085313610,116.433339390634,336.754822217052,-0.00801228071276919}} ;

// Variables for Kalman Filter for Slip Estimation
float slip_ratio_corr[2] = {0.7,0.7};
float pred_slip_ratio[2] = {0.0,0.0};
float slip_ratio[2];  // avg slip ratios after 4 ms
float slip_ratio_right_arr[4] = {0.0,0.0,0.0,0.0};
float slip_ratio_left_arr[4] = {0.0,0.0,0.0,0.0};
float pred_P_slip[2];
float P_slip[2] = {30.0,30.0};
float Q_slip[2] = {0.01,0.01};
float S_slip[2] = {0.0,0.0};
float R_slip[2] = {50000,50000};
float W[2]; // Kalman Gain
float residual[2];

// New Balance Control Variables
float a1 = 0.0;
float a2 = 0.0;
float a3 = 0.0;
float a4 = 0.0;
float bal_states_des[4] = {0.0,0.0,0.0,0.0};
float bal_states[4]  = {0.0,0.0,0.0,0.0};
//float K1ns[2][4] = {{-1.02056065905144e-05, 0.000187888065971112,    -0.000348541576804817,   -2327330.38192134},
//                  {8.65307635122310e-05,    44.9129835002238,    2327375.29501721,    -0.00721309002194323}};
//float K2ns[2][4] = {{2.92442456283251e-06,   -17.8804910994032,   581763.469532093,   -1163767.68158941},
//                  {0.000130813317911382,    80.6728070217384,    1163856.02509364,    -2327110.06928085}};
//float K3ns[2][4] = {{-1.35585611223362e-05,  17.8806868934597,    -581763.469866915,   -1163767.68168305},
//                  {0.000109573631663982,    80.6731979772495,    1163856.02442445,    2327110.05305911}};
//float K4ns[2][4] = {{0.0, 0.0, 0.0, -204.981370868571},
//                  {0.000136590085313610, 116.433339390634, 336.754822217052,    -0.00801228071276919}};
float K1ns[2][4] = {{3.6920, 0.3939,    -0.0210,   0.0},
                    {0.0,    0.0,    0.0,    0.0191}};
float K2ns[2][4] = {{4.5530,   881,   -0.0280,   0.0},
                    {0.0,0.0,  0.0,   0.0203}};
float K3ns[2][4] = {{2.4680,  0.2624, -0.0130,  0.0},
                    {0.0,  0.0,  0.0, 0.0203}};
float K4ns[2][4] = {{2.468, 0.2624, -0.0130, 0.0},
                    {0.0, 0.0, 0.0, 0.0131}};
//float K1ns[2][4] = {{-60,    -4.5,  -1.1, -0.1},
//                  {0.0,    0.0,    0.0,    0.0191}};
//float K2ns[2][4] = {{-60,    -4.5,  -1.1, -0.1},
//                  {0.0,    0.0,    0.0,    0.0191}};
//float K3ns[2][4] = {{-60,    -4.5,  -1.1, -0.1},
//                  {0.0,    0.0,    0.0,    0.0191}};
//float K4ns[2][4] = {{-60,    -4.5,  -1.1, -0.1},
//                  {0.0,    0.0,    0.0,    0.0191}};
float meas_states_k[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
float meas_states_k_1[6]= {0.0,0.0,0.0,0.0,0.0,0.0};
float ubalns[2] = {0.0,0.0};
float ubal_torque = 0.0;
float ubal_diff_torque = 0.0;

float w = 0.568;   // Body width [ft]
float Rad = 0.1946; // Radius of wheels [ft]
float err[4] = {0.0,0.0,0.0,0.0}; // Error between desired and measured/estimated states
float WhlPosR_k, WhlPosL_k, WhlPosR_k_1, WhlPosL_k_1; // Wheel Positions measured every 1 ms
float WhlSpdL_k, WhlSpdR_k, WhlSpdR_k_1, WhlSpdL_k_1;
float f_left_k, f_right_k, f_right_k_1, f_right_k_1; //// Friction estimates every 1 ms
float bal_states[4]; // states for the balance control
float bal_states_des[4]; // our desired state values
float meas_states_arr[4][4]; // states for the observer model
float a[4] = {0.0,0.0,0.0,0.0}; // slip ratios
float torque_com, torque_diff_com;
float turnrate_ref_k;
float turnrate_ref_k_1;
float turn_k_1;
void setEPWM2A(float controleffort) {
    if (controleffort > 10) {
        controleffort = 10;
    } else if (controleffort < -10) {
        controleffort = -10;
    }
    float toSet = (controleffort + 10.0) * 125.0;
    EPwm2Regs.CMPA.bit.CMPA = toSet;
}

void setEPWM2B(float controleffort) {
    if (controleffort > 10) {
        controleffort = 10;
    } else if (controleffort < -10) {
        controleffort = -10;
    }
    float toSet = (controleffort + 10.0) * 125.0;
    EPwm2Regs.CMPB.bit.CMPB = toSet;
}

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (-raw/1909.86); //raw divided by (400*30/(2pi))
}

float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the v
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw/1909.86);
}

void StateMachine(void){
    time_inc++;
    switch(stateVar){
    case 10: // when line is lost, turn until line is found
    {
        //            if(time_inc <= 2000) {
        //                speeddes = -0.5;
        //                turn = 0.0;
        //            } else {
        //                speeddes = 0.0;
        //                turn = 0.5;
        //            }

        speeddes = -0.7;
        turn = 0.0;

        if(obstacle == 1){
            time_inc = 0;
            stateVar = 20;
        }
        else if(is_line == 1){ //If under fixed distance threshold switch to obstacle avoidance state
            time_inc = 0;
            stateVar = 20;
        }
        //            if (time_inc == 200){ //Check to see if we are near obstacle every 200ms
        //                if(min_dist <= stop_dis){ //If under fixed distance threshold switch to obstacle avoidance state
        //                    time_inc = 0;
        //                    stateVar = 30;
        //                }
        //                while (is_line == 0){ //If line is lost then turn until line is found
        //                    turnref = 3.14*0.1;
        //                }
        //            }
        break;
    }
    case 20: //Line following state until line is lost
    {
        line_turn_f = (float)line_turn;
        turn = (line_turn_f-128.0) / 25.0;
        speeddes = 1;
        if(obstacle == 1){
            time_inc = 0;
            stateVar = 30;
        }
        else if(is_line == 0){ //When line is lost switch to line lost state
            time_inc = 0;
            stateVar = 10;
        }

        //            if(min_dist <= stop_dis){
        //                time_inc = 0;
        //                stateVar = 30;
        //            }

        break;
    }
    case 30: //Avoid Obstacle routine
    {
        //            speeddes = -1.0;
        if(obstacle == 1) {
            turn = 0.0;
            speeddes = -(max_ir-1500.0)/500.0; //Prox values will range from 2k to 3k; this while convert to -1 to -3
            if(speeddes > -0.5) { // Make sure to keep moving backwards
                speeddes = -0.5;
            }
        } else if (is_line){
            time_inc = 0;
            stateVar = 20;
        } else {
            time_inc = 0;
            stateVar = 10;
        }

        //            if(time_inc == 200){ //turn from 200-1200ms
        //                turnref = 3.14*0.1;
        //            }
        //            if(time_inc >= 1200 && time_inc <= 2200){ //stop at 1200ms
        //                turnref = 0.0;
        //                if(min_dist >= stop_dis){ //If obstacle is detected then stay in this state
        //                    time_inc = 0;
        //                    stateVar = 30;
        //                }else{ ///If no obstacle detected go backwards
        //                    speeddes = -1;
        //                }
        //            }
        //            if(time_inc == 2200){ //Once we reach 2200ms in this function we want to
        //                speeddes = -0.1;
        //                turnref = -3.14*0.1;
        //            }
        //            if (time_inc == 2700){
        //                turnref = 0;
        //                if(min_dist>=stop_dis){
        //                    time_inc = 0;
        //                    stateVar = 30;
        //                }else{
        //                    speeddes = 1;
        //                }
        //            }
        //            if (time_inc == 3400){
        //                speeddes = 0.1;
        //                turnref = 3.14*0.1;
        //
        //                if (time_inc == 3900){
        //                    speeddes = 1;
        //                    turnref = 0;
        //                }
        //                if(time_inc == 4800){
        //                    time_inc = 0;
        //                    stateVar = 10;
        //                }
        //
        break;
    }

    }
}



float BalanceNoSlip(void){
    // An Observer and Kalman Filter predict slip of the wheels and compensates for it by adjusting
    // the balance control law with coefficients called slip ratios

    //Slip ratios
    a[0] = (1.0-slip_ratio[0])*(1.0-slip_ratio[1]);
    a[1] = (slip_ratio[0])*(1.0-slip_ratio[1]);
    a[2] = (1.0- slip_ratio[0])*slip_ratio[1];
    a[3]= slip_ratio[0]*slip_ratio[1];

    bal_states_des[0] = 3.14; // We want the SegBot to be upright by 180 degrees [rad]
    bal_states_des[1] = 0.0;  // We want no tilt speed
    bal_states_des[2] = speeddes/Rad; // desired angular speed [rad/s] given linear speed input speeddes [ft/s]

    turnrate_ref_k = 0.6*turnrate_ref_k_1 + 100*turn - 100*turn_k_1;
    //given a turn command from the OpenMV turn rate
    bal_states_des[3] = (w*turnrate_ref_k)/Rad; // desired angular wheel diff

    for (int i=0; i < 4; i++){
        err[i] = bal_states[i]-bal_states_des[i];
    }

    torque_com= 0.0;
    torque_diff_com = 0.0;
    for (int i=0; i<4; i++){
        torque_com = torque_com + (a[0]*K1ns[0][i] + a[1]*K2ns[0][i] + a[2]*K3ns[0][i] + a[3]*K4ns[0][i])*(err[i]);
        torque_diff_com = torque_diff_com + (1.0*a[0]*K1ns[1][i] + a[1]*K2ns[1][i] + a[2]*K3ns[1][i]+ a[3]*K4ns[1][i])*(err[i]);
    }

    ubalns[0] = torque_com;
    ubalns[1] = torque_diff_com;

    turnrate_ref_k_1 = turnrate_ref_k;

    return ubalns[0]+ ubalns[1];
}



// Three interrupt functions were created for three converters, ADCA, ADCB, and ADCD.
// The interrupt functions allow the results registers to be read
// for the most recent converted sample at a specific sample rate

// adcb1 pie interrupt
// Sample and filter the frequency input from the microphone signal
__interrupt void ADCD_ISR (void)
{
    adcd0result = AdcdResultRegs.ADCRESULT0;
    adcd1result = AdcdResultRegs.ADCRESULT1;
    adcd2result = AdcdResultRegs.ADCRESULT2;

    max_ir = adcd0result;
    if(adcd1result > max_ir){
        max_ir = adcd1result;
    }
    if(adcd2result > max_ir){
        max_ir = adcd2result;
    }
    if(max_ir >= stop_dis){
        obstacle = 1;
    } else {
        obstacle = 0;
    }

    //    if(adcd0result >= stop_dis){
    //        obstacle = 1;
    //    } else if(adcd1result >= stop_dis){
    //        obstacle = 1;
    //    } else if(adcd2result >= stop_dis){
    //        obstacle = 1;
    //    } else {
    //        obstacle = 0;
    //    }
    // dis_count++;


    if ((numADCD1InterruptCalls % 400) == 0) {
        UARTPrint = 1;
    }

    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //clear PIE peripheral
    numADCD1InterruptCalls++;
}


void main(void){
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    DELAY_US(100000);
    // RESET ESP8266
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
    DELAY_US(500000);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    DELAY_US(3000000);


    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();


    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR; //Exercise 1
    PieVectTable.ADCD1_INT = &ADCD_ISR;  // When ADCD1_INT is triggered ADCD_ISR is called

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)

    //Exercise 3: changed to every 20 ms; Exercise 4: changed to every 1ms
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 5000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    init_serialSCIC(&SerialC,115200);

    // Exercise 4: Add setupSpib() call
    setupSpib();

    // Lab 5 Exercise 1:
    init_eQEPs();

    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 1250;
    EPwm2Regs.CMPB.bit.CMPB = 1250;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    // ADC Setup Code
    // EPwm5 PRD time event is used to trigger the start of analog to digital conversion
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3;    // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2;    // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm5Regs.TBCTR = 0x0;              // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0;  // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period for ADCD to 1ms sample. Input clock is 50MHz. 1ms period is 1Khz. (50Mhz / 1Khz) = 50,000

    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1;  // enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; // unfreeze, and enter up count mode
    EDIS;

    EALLOW;

    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;    // SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99;    // sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;     //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99;    //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB
    //    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4 ;      //SOC0 will convert Channel 4
    //    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99;      //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;   // EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???;   //SOC1 will convert Channel you choose Does not have to be B1
    //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99;    //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???;   //SOC2 will convert Channel you choose Does not have to be B2
    //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99;    //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???;   //SOC3 will convert Channel you choose Does not have to be B3
    //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99;    //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    //    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // set to last SOC that is converted, for this converter we only read one channel
    //    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    //    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCD
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;     // set SOC0 to convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99;    //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC0
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;     //set SOC1 to convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99;    //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC1
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 2;   //set SOC2 to convert pin D2
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99;    //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???;   //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99;    //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 2; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;


    //    // Lab 7 Initialize EPWM5 Register
    //    EALLOW;
    //    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    //    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    //    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    //    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (Â“pulseÂ” is the same as Â“triggerÂ”)
    //    EPwm5Regs.TBCTR = 0x0; // Clear counter
    //    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    //    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    //    // Set CLKDIV to 2 to achieve period of 0.25 ms
    //    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    //    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    //    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    //    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    ////    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    //    EDIS;
    //
    //    //Initialized ADCA
    //    EALLOW;
    //    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    //    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    //    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //    //Set pulse positions to late
    //    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //    //power up the ADCs
    //    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //    //delay for 1ms to allow ADC time to power up
    //    DELAY_US(1000);
    //
    //    //Exercise 3: Provided code to setup each ADC
    //    //Select the channels to convert and end of conversion flag
    //    //Many statements commented out, To be used when using ADCA or ADCB
    //    //ADCA
    //    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be A0
    //    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be A1
    //    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    //    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    //    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //    EDIS;

    // Set GPIO pins to correspond with mux selections of EPWM2A/B
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index

    // Dan moved here so the ADCA trigger starts from this piont
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT6; //Exercise 2: enable group 6 for SPIB
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable SPIB in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    // Enable ADCA1 in the PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    // Enable PIE interrupt for ADCD1
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    serial_sendSCIC(&SerialC,"AT\r\n",strlen("AT\r\n"));

    displayLEDletter(0);
    ESP8266whichcommand = 1;
    ESP8266insidecommands = 1;

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //            serial_printf(&SerialA,"Joystick 1:%.3f Joystick 2:%.3f AccelZ:%.3f GyroX:%.3f LeftWheel:%.3f RightWheel:%.3f \r\n",voltD0,voltD1,accelz,gyrox,LeftWheel,RightWheel);
            serial_printf(&SerialA,"s1:%d s2:%d s3:%d\r\n",adcd0result,adcd1result,adcd2result);
            UARTPrint = 0;
        }
    }
}

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;

int16_t status = 0;
//int16_t axout = 0;
//int16_t ayout = 0;
//int16_t azout = 0;
//int16_t temp = 0;
//int16_t gxout = 0;
//int16_t gyout = 0;
//int16_t gzout = 0;

//adca1 pie interrupt
__interrupt void ADCA_ISR (void)
{
    rawD0 = AdcaResultRegs.ADCRESULT0;
    rawD1 = AdcaResultRegs.ADCRESULT1;

    // Here covert ADCIND0 to volts
    voltD0 = rawD0 * 3.0 / 4096;
    voltD1 = rawD1 * 3.0 / 4096;

    // Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x00DA; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //    SpibRegs.SPITXBUF = 1500; // something so you can see the pattern on the Oscilloscope
    //    SpibRegs.SPITXBUF = 1000;

    // 0xBA00 = (0x3A00 | 0x8000), which indicates a 1 for reading and a 3A for the address
    SpibRegs.SPITXBUF = (0xBA00 | 0x0000); // Writes 0 to 3A
    SpibRegs.SPITXBUF = 0x0000; // Writes 0 to 3B and 3C (Accel_Xout)
    SpibRegs.SPITXBUF = 0x0000; // Writes 0 to 3D and 3E (Accel_Yout)
    SpibRegs.SPITXBUF = 0x0000; // Writes 0 to 3F and 40 (Accel_Zout)
    SpibRegs.SPITXBUF = 0x0000; // Writes 0 to 41 and 42 (Temp)
    SpibRegs.SPITXBUF = 0x0000; // Writes 0 to 43 and 44 (Gyro_Xout)
    SpibRegs.SPITXBUF = 0x0000; // Writes 0 to 45 and 46 (Gyro_Yout)
    SpibRegs.SPITXBUF = 0x0000; // Writes 0 to 47 and 48 (Gyro_Zout)

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void SPIB_isr(void) {
    status = SpibRegs.SPIRXBUF; // Read int status
    raw_accelx = SpibRegs.SPIRXBUF; // Read accel_xout
    raw_accely = SpibRegs.SPIRXBUF;
    raw_accelz = SpibRegs.SPIRXBUF;
    raw_temp = SpibRegs.SPIRXBUF;
    raw_gyrox = SpibRegs.SPIRXBUF;
    raw_gyroy = SpibRegs.SPIRXBUF;
    raw_gyroz = SpibRegs.SPIRXBUF;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPI66 high to end Slave Select. Now Scope. Later to deselect DAN28027

    accelx = raw_accelx / 32768.0 * 4.0;
    accely = raw_accely / 32768.0 * 4.0;
    accelz = raw_accelz / 32768.0 * 4.0;
    gyrox = raw_gyrox / 32768.0 * 250.0;
    gyroy = raw_gyroy / 32768.0 * 250.0;
    gyroz = raw_gyroz / 32768.0 * 250.0;

    //    LeftWheel = readEncLeft(); // This is angular distance traveled in radians
    //    RightWheel = readEncRight();

    // Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    // The Segbot's blue LED blinks while the Segbot powers on for two seconds and calibrates for to two seconds.
    // The offsets are the average of the measurements taken in last two seconds. For this reason, let the Segbot sit still
    // until the blue LED no longer blinks.

    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        // The Kalman filter predicts tilt by integrating tiltrate read from the gyroscope with the forward rule.
        // The variance increases by q and it is represents the inverse of trust. We are losing trust with each measurement during dead reckoning. Q is set to 0.01.
        // We update the predicted value with the accelerometer reading. "Trust" in the accelerometer reading is represented by R, set to 25000.
        // The tilt value is updated with the kalman gain times the error between the measurement and the prediction.
        // Finally, average the filter values after 4 reads as the balancing control law works every 4 ms.

        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt +  T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ. This is technically in radians. The small angle rule applies.
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;

        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();

        /*--------------State observer code start---------------------------------------------------------------------*/
        // Here begins an observer for traction of the wheels, f_left and f_right:
        // 6th order where the states are tilt, tilt rate, relative wheel speed, differential wheel speed, left wheel traction, and right wheel traction]

        WhlPosR_k = RightWheelArray[SpibNumCalls];
        WhlPosL_k = LeftWheelArray[SpibNumCalls];

        // A derivative approximation estimates angular wheel speed from wheel position
        WhlSpdL_k = 0.6*WhlSpdL_k_1 + 100*WhlPosR_k - 100*WhlPosR_k_1;
        WhlSpdR_k = 0.6*WhlSpdR_k_1 + 100*WhlPosL_k - 100*WhlPosL_k_1;

        // The first four states are measured. We already implemented sensor fusion to calculate tilt.
        meas_states_k[0] = kalman_tilt; // Tilt angle
        meas_states_k[1] = tiltrate; // Rate of tilt
        meas_states_k[2] = (WhlSpdL_k+WhlSpdR_k)*0.5+tiltrate; // Average wheel speed relative to tilt rate
        meas_states_k[3] = WhlSpdL_k-WhlSpdR_k; //Differential Wheel Speed

        // Estimate friction on each wheel from known states and balance command
        f_left_k = 0.0;
        f_right_k = 0.0;

        for (int i=0; i<6; i++){
            f_left_k = f_left_k + E[5][i]*meas_states_k[5];
            f_right_k = f_right_k + E[4][i]*meas_states_k[4];
        }

        for (int i=0; i<2; i++){
            f_left_k = f_left_k + Bz[5][i]*ubalns[i];
            f_right_k= f_left_k + Bz[4][i]*ubalns[i];
        }

        for (int i=0; i<4; i++){
            f_left_k= f_left_k-L[5][i]*meas_states_k[i];
            f_right_k= f_right_k-L[4][i]*meas_states_k[i];
        }

        meas_states_k[4] = f_right_k;
        meas_states_k[5] = f_left_k;

        for (int i=0; i<4; i++){
            meas_states_arr[i][SpibNumCalls] = meas_states_k[i];
        }

        /*--------------Kalman Filter for Slip Estimation Code Start---------------------------------------------------------------------*/
        // This Kalman filter uses friction from the observer to estimate slip
        // Assumes slip ratio is a constant so it appears as process disturbance
        // I make a guess for my prediction

        // Prediction Step
        pred_slip_ratio[0] = slip_ratio_corr[0];
        pred_slip_ratio[1]= slip_ratio_corr[1];
        pred_P_slip[0] = P_slip[0] + Q_slip[0];
        pred_P_slip[1] = P_slip[1] + Q_slip[1];

        // Update Step
        // The ideal traction is calculated when slip velocity is zero
        // ideal traction times the slip ratio should equal the estimated traction
        // I want to improve the slip ratio estimate by comparing estimated and predicted traction
        // Apparently dividing directly causes numerical instability

        float f_right_ideal = 0.0;
        float f_left_ideal = 0.0;


        for (int i = 0; i<4; i++){
            f_right_ideal = f_right_ideal + M1[1][i]*meas_states_k[i];
            f_left_ideal = f_left_ideal + M1[2][i]*meas_states_k[i];
        }

        for (int i = 0; i<2; i++){
            f_right_ideal = f_right_ideal + M2[1][i]*ubalns[i];
            f_left_ideal = f_left_ideal + M2[2][i]*ubalns[i];
        }

        residual[0] = pred_slip_ratio[0]*f_right_ideal - meas_states_k[4];
        residual[1] = pred_slip_ratio[1]*f_left_ideal - meas_states_k[5];

        for (int i=0; i<2;i++){
            S_slip[i] = pred_P_slip[i] + R_slip[i];
            W[i] = pred_P_slip[i]/S_slip[i];
            slip_ratio_corr[i] = pred_slip_ratio[i] - W[i]*residual[i];

            slip_ratio_right_arr[SpibNumCalls] = slip_ratio_corr[0];
            slip_ratio_left_arr[SpibNumCalls] = slip_ratio_corr[1];

            // Save Previous States
            WhlPosL_k_1 = WhlPosL_k;
            WhlPosR_k_1 = WhlPosR_k;
            WhlSpdL_k_1 = WhlSpdL_k;
            WhlPosR_k_1 = WhlSpdR_k;

            if (SpibNumCalls >= 3) { // should never be greater than 3
                // average my measured states
                bal_states[0] = (meas_states_arr[0][0]+meas_states_arr[0][1]+meas_states_arr[0][2]+meas_states_arr[0][3])/4.0;
                bal_states[1] = (meas_states_arr[1][0]+meas_states_arr[1][1]+meas_states_arr[1][2]+meas_states_arr[1][3])/4.0;
                bal_states[2] = (meas_states_arr[2][0]+meas_states_arr[2][1]+meas_states_arr[2][2]+meas_states_arr[2][3])/4.0;
                bal_states[3] = (meas_states_arr[3][0]+meas_states_arr[3][1]+meas_states_arr[3][2]+meas_states_arr[3][3])/4.0;
                slip_ratio[0] = (slip_ratio_right_arr[0]+slip_ratio_right_arr[1]+slip_ratio_right_arr[2]+slip_ratio_right_arr[3])/4.0;
                slip_ratio[1] = (slip_ratio_left_arr[0]+slip_ratio_left_arr[1]+slip_ratio_left_arr[2]+slip_ratio_left_arr[3])/4.0;

                tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0; // same as balancing state one
                gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0; // same as balancing state two
                LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
                RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
                SpibNumCalls = -1;
                PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
            }
        }
        timecount++;
        if((timecount%200) == 0)
        {
            if(doneCal == 0) {
                GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
            }
            GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
            UARTPrint = 1; // Tell While loop to print
        }

        SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
        SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    }

}

    // Lab 7 exercises 1 through 4
    // Implement balance, steering, and driving control to the Segbot using sensor feedback

    // SWI_,  Using this interrupt as a Software started interrupt
    __interrupt void SWI_isr(void) {

        // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
        // making it lower priority than all other Hardware interrupts.
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
        asm("       NOP");                    // Wait one cycle
        EINT;                                 // Clear INTM to enable interrupts

        // A derivative approximation estimates wheel speed
        // Wheel speed is theta_dot_k abbreviate as thdk_L and thdk_R
        thdLK = 0.6*thdLK_1 + 100*LeftWheel - 100*LeftWheelK_1;
        thdRK = 0.6*thdRK_1 + 100*RightWheel - 100*RightWheelK_1;
        gyrorate_dot = 0.6*gyrorate_dotK_1 + 100*gyro_value - 100*gyro_valueK_1;

        // States are saved
        LeftWheelK_1 = LeftWheel;
        RightWheelK_1 = RightWheel;
        thdLK_1 = thdLK;
        thdRK_1 = thdRK;
        gyro_valueK_1 = gyro_value;
        gyrorate_dotK_1 = gyrorate_dot;


        // Lab 7 Ex 4
        // Another derivative approximation is used to estimate average differential wheel speed
        // Turning is managed by a simple PID controller
        // Note we control turn rate but integrate (trapezoidal integration) to get a turn reference

        WhlDiff = LeftWheel - RightWheel;
        vel_WhlDiff = 0.333*vel_WhlDiffK_1 + 166.667*WhlDiff - 166.667*WhlDiffK_1;
        // errorDiff = turnref - WhlDiff;
        // errorDiff = turnref;
        // intDiff = intDiffK_1 + 0.004*(errorDiff + errorDiffK_1)/2;
        // turn = Kp*errorDiff + Ki*intDiff;
        // turnref = turnrefK_1 + 0.004*(turnrate + turnrateK_1)/2;

        //    if (is_line) {
        //        turn = (line_turn_f-128.0) / 25.0;
        //        speeddes = 1;
        //    } else {
        //        speeddes = -0.1;
        //        turn = 0.0;
        //    }
        //    if (fabs(turnref) > 3.0) {
        //        turnref = turnrefK_1;
        //    }

        //Fwdbackoffset control
        // We implement one more PID controller for forward and backward control
        //    vel_fwb = 0.333*vel_fwbK_1 + 166.667*fwb - 166.667*fwbK_1

        errorSpeed = speeddes - (thdLK + thdRK)/2;
        intSpeed = intSpeedK_1 + 0.004*(errorSpeed + errorSpeedK_1)/2;
        FwdBackOffset = sKp*errorSpeed + sKi*intSpeed;

        //    errorDiff = turnref - WhlDiff;
        //    intDiff = intDiffK_1 + 0.004*(errorDiff + errorDiffK_1)/2;
        //    turn = Kp*errorDiff + Ki*intDiff - Kd*vel_WhlDiff;
        //    turnref = turnrefK_1 + 0.004*(turnrate + turnrateK_1)/2;

        //integral windup check
        if (fabs(turn) > 3.0) {
            intDiff = intDiffK_1;
        }
        if (fabs(FwdBackOffset) > 3.0) {
            intSpeed = intSpeedK_1;
        }

        // Saturate control signals
        if (turn > 4.0) {
            turn = 4.0;
        } else if (turn < -4.0) {
            turn = -4.0;
        }

        if (FwdBackOffset > 4.0) {
            FwdBackOffset = 4.0;
        } else if (FwdBackOffset < -4.0) {
            FwdBackOffset = -4.0;
        }

        // The control command is centered around the balance controller adjusted by time and forward/backward offset
        // Balance has priority so turning and forward/backward control signals are saturated above.
        // The control law describes a full state feedback controller with the highest gain on the tilt value, expecting the most agressive control on tilt
        // gyrorate_dot has a fractional gain, perhaps attenuates the response to tilt acceleration

        ubal = -K1*tilt_value - K2*gyro_value - K3*(thdLK+thdRK)/2 - K4*gyrorate_dot;

        /*------------------------No Slip Balance Control----------------------*/
        //  ubal = BalanceNoSlip()
        /*------------------------No Slip Balance Control---------------------*/

        uRight = ubal/2.0 - turn - FwdBackOffset;
        uLeft = ubal/2.0 + turn - FwdBackOffset;

        setEPWM2A(uRight);
        setEPWM2B(-uLeft);

        // Finally, save remaining states
        errorDiffK_1 = errorDiff;
        vel_WhlDiffK_1 = vel_WhlDiff;
        intDiffK_1 = intDiff;
        turn_k_1 = turn;
        turnrateK_1 = turnrate;
        errorSpeedK_1 = errorSpeed;
        intSpeedK_1 = intSpeed;

        numSWIcalls++;

        DINT;

    }

    // cpu_timer0_isr - CPU Timer0 ISR
    __interrupt void cpu_timer0_isr(void)
    {
        CpuTimer0.InterruptCount++;

        numTimer0calls++;

        //    if (count == 0) {
        //        toAdd = 10;
        //    } else if (count == 3000) {
        //        toAdd = -10;
        //    }
        //    count += toAdd;
        //    SpibRegs.SPITXBUF = count;
        //    SpibRegs.SPITXBUF = 3000-count;

        //    if ((numTimer0calls%50) == 0) {
        //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
        //    }

        if ((numTimer0calls%250) == 0) {
            //        displayLEDletter(LEDdisplaynum);
            LEDdisplaynum++;
            if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
                LEDdisplaynum = 0;
            }
        }



        // Blink LaunchPad Red LED
        //    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

        // Acknowledge this interrupt to receive more interrupts from group 1
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

        //    if ((CpuTimer0.InterruptCount % 100) == 0) {
        //        UARTPrint = 1;
        //    }


        //    s0_avg += adcd0result*0.1;
        //    s1_avg += adcd1result*0.1;
        //    s2_avg += adcd2result*0.1;
        //    if((dis_count%10) == 0 ){
        //        min_dist = s0_avg;
        //        if (min_dist > s1_avg){
        //            if((s1_avg - s2_avg)>0){
        //                min_dist = s2_avg;
        //            }else{
        //                min_dist = s1_avg;
        //            }
        //        }
        //        s0_avg =0.0;
        //        s1_avg =0.0;
        //        s2_avg =0.0;
        //    }


    }

    // cpu_timer1_isr - CPU Timer1 ISR
    __interrupt void cpu_timer1_isr(void)
    {
        LeftWheelL6 = readEncLeft(); // This is angular distance traveled in radians
        RightWheelL6 = readEncRight();
        //    LeftTravel = LeftWheelL6 / 5.1; // In feet, this value is currently unused
        //    RightTravel = RightWheelL6 / 5.1; // Multiply Encoder Reading by (1/(2pi) * 2*pi*r
        //
        //    PosLeft_K = LeftTravel;
        //    PosRight_K = RightTravel;
        //    // Velocity = Difference in position over time step
        //    VLeftK = (PosLeft_K - PosLeft_K_1)/.004;
        //    VRightK = (PosRight_K - PosRight_K_1)/.004;
        //
        //    eturn = turn + (VLeftK - VRightK);
        //    eLeftK = Vref - VLeftK - Kturn*eturn;
        //    eRightK = Vref - VRightK + Kturn*eturn;
        //
        //    ILeftK = ILeftK_1 + 0.004*(eLeftK+eLeftK_1)/2;
        //    IRightK = IRightK_1 + 0.004*(eRightK+eRightK_1)/2;
        //
        //    uLeftK = Kp*eLeftK + Ki*ILeftK;
        //    uRightK = Kp*eRightK + Ki*IRightK;
        //
        //    if(uLeftK > 10 || uLeftK < -10) {
        //        ILeftK = ILeftK_1;
        //        uLeftK = Kp*eLeftK + Ki*ILeftK;
        //    }
        //    if(uRightK > 10 || uRightK < -10) {
        //        IRightK = IRightK_1;
        //        uRightK = Kp*eRightK + Ki*IRightK;
        //    }
        //
        //    setEPWM2A(uRightK);
        //    setEPWM2B(-uLeftK); //Switch directions so positive uleft is forwards
        //
        //    // Do pose calculations here
        WR = 0.56759;
        RWh = 0.19460;
        theta_r = RightWheelL6;
        theta_l = LeftWheelL6;
        bearing = ((RWh/WR)*(theta_r-theta_l));
        theta_avg = 0.5*(theta_r+theta_l);
        t_dot_r = (theta_r - theta_r_1)/0.004;
        t_dot_l = (theta_l - theta_l_1)/0.004;
        t_dot_avg = 0.5*(t_dot_r + t_dot_l); // Need to defined t_dots still
        x_dot_R = RWh*t_dot_avg*cos(bearing);
        y_dot_R = RWh*t_dot_avg*sin(bearing);
        xRK = xRK_1 + 0.004*(x_dot_R + x_dot_R_1)/2;
        yRK = yRK_1 + 0.004*(y_dot_R + y_dot_R_1)/2;

        // Code to give absolute direction in degrees
        if (bearing > PI) {
            bearing_mod = bearing - 2.0*PI*floor((bearing+PI)/(2*PI));
        } else if (bearing < -PI) {
            bearing_mod = bearing - 2.0*PI*ceil((bearing-PI)/(2*PI));
        } else {
            bearing_mod = bearing;
        }
        bearing_mod = bearing_mod/(2*PI)*360;

        //Save old states
        PosLeft_K_1 = PosLeft_K;
        PosRight_K_1 = PosRight_K;
        eLeftK_1 = eLeftK;
        eRightK_1 = eRightK;
        ILeftK_1 = ILeftK;
        IRightK_1 = IRightK;
        theta_r_1 = theta_r;
        theta_l_1 = theta_l;
        xRK_1 = xRK;
        yRK_1 = yRK;
        x_dot_R_1 = x_dot_R;
        y_dot_R_1 = y_dot_R;

        CpuTimer1.InterruptCount++;
    }

    // cpu_timer2_isr CPU Timer2 ISR
    __interrupt void cpu_timer2_isr(void)
    {
        // Blink LaunchPad Blue LED
        //    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

        CpuTimer2.InterruptCount++;

        //  if ((CpuTimer2.InterruptCount % 50) == 0) {
        //      UARTPrint = 1;
        //  }

        // Call state machine every 5ms
        StateMachine();
    }

    void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
    {
        //    int16_t temp = 0;
        //    Step 1.
        // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
        //    between each transfer to 0. Also donÂ’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
        //    66 which are also a part of the SPIB setup.

        //Exercise 2: Code provided
        GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
        GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
        GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
        GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
        GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
        GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
        GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
        GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
        GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
        EALLOW;
        GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
        GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
        GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
        GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
        GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
        GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
        EDIS;
        // ---------------------------------------------------------------------------
        SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
        SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
        SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
        SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
        SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16 bits each write to SPITXBUF
        SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
        SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
        SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
        SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49 ; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
        // 50MHZ. And this setting divides that base clock to create SCLKÂ’s period
        SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
        SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
        SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
        SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
        SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
        SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
        SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
        SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
        SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
        SpibRegs.SPIFFCT.bit.TXDLY = 16 ; //Set delay between transmits to 16 spi clocks. Needed by DAN28027

        SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
        SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
        SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
        SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I donÂ’t think this is needed. Need to Test
        SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes
        //interrupt. This is just the initial setting for the register. Will be changed below

        SpibRegs.SPICCR.bit.SPICHAR = 0xF;
        SpibRegs.SPIFFCT.bit.TXDLY = 0x00;

        // Step 2.

        // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
        // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
        // some code is given, most you have to fill you yourself.
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

        // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are
        //sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.

        //    SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Not sure if we need this?

        // To address 00x13 write 0x00
        // To address 00x14 write 0x00
        // To address 00x15 write 0x00
        // To address 00x16 write 0x00
        // To address 00x17 write 0x00
        // To address 00x18 write 0x00
        // To address 00x19 write 0x13
        // To address 00x1A write 0x02
        // To address 00x1B write 0x00
        // To address 00x1C write 0x08
        // To address 00x1D write 0x06
        // To address 00x1E write 0x00
        // To address 00x1F write 0x00

        SpibRegs.SPITXBUF = (0x1300 | 0x0000);
        SpibRegs.SPITXBUF = (0x0000 | 0x0000);
        SpibRegs.SPITXBUF = (0x0000 | 0x0000);
        SpibRegs.SPITXBUF = (0x0000 | 0x0013);
        SpibRegs.SPITXBUF = (0x0200 | 0x0000);
        SpibRegs.SPITXBUF = (0x0800 | 0x0006);
        SpibRegs.SPITXBUF = (0x0000 | 0x0000);

        // wait for the correct number of 16 bit values to be received into the RX FIFO
        while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF; // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
        DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

        //    Step 3.
        // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
        // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
        // some code is given, most you have to fill you yourself.
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
        // Perform the number of needed writes to SPITXBUF to write to all 7 registers
        // To address 00x23 write 0x00
        // To address 00x24 write 0x40
        // To address 00x25 write 0x8C
        // To address 00x26 write 0x02
        // To address 00x27 write 0x88
        // To address 00x28 write 0x0C
        // To address 00x29 write 0x0A
        SpibRegs.SPITXBUF = (0x2300 | 0x0000);
        SpibRegs.SPITXBUF = (0x4000 | 0x008C);
        SpibRegs.SPITXBUF = (0x0200 | 0x0088);
        SpibRegs.SPITXBUF = (0x0C00 | 0x000A);

        // wait for the correct number of 16 bit values to be received into the RX FIFO
        while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF;
        temp = SpibRegs.SPIRXBUF; // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
        DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

        //    Step 4.
        // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A

        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        // Write to address 0x2A the value 0x81
        SpibRegs.SPITXBUF = (0x2A00 | 0x0081);
        // wait for one byte to be received
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);

        // The Remainder of this code is given to you and you do not need to make any changes.
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500

        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7700 | 0x0012); // 0x7700
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7800 | 0x00FA); // 0x7800
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7A00 | 0x0011); // 0x7A00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7B00 | 0x00E2); // 0x7B00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7D00 | 0x001B); // 0x7D00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(10);
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
        SpibRegs.SPITXBUF = (0x7E00 | 0x00AC); // 0x7E00
        while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        temp = SpibRegs.SPIRXBUF;
        DELAY_US(50);
        // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
        SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
        SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
    }
