/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "Serial.h"
#include "PwmIn.h"
#include "rtos.h"
#include "QEI.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"
// #include "FutabaSBUS.h"

// Blinking rate in milliseconds
#define BLINKING_RATE     1000

DigitalOut led(LED1);
DigitalOut digital_out1(D0);

// #define X8R_RADIO_BUF_LEN 25
// #define NUM_X8R_RADIO_CHANNELS 18
// #define X8R_RADIO_BAUDRATE 100000
// #define X8R_RADIO_NUM_BITS 8
// #define X8R_RADIO_PARITY SerialBase::Even
// #define X8R_RADIO_NUM_STOP_BITS 2

Serial pc(USBTX, USBRX);  // tx, rx

// Serial radio(PD_5, PD_6);  // tx, rx
// uint16_t read_buff[X8R_RADIO_BUF_LEN] = {0};
// uint16_t channels[NUM_X8R_RADIO_CHANNELS];
// uint16_t channel_data[NUM_X8R_RADIO_CHANNELS];

//  FutabaSBUS sbus(PD_5, PD_6);

#define NUM_RC_CHANNELS 4
PwmIn ch1(A1);
PwmIn ch2(A3);
PwmIn ch3(A4);
PwmIn ch4(A5); // find a new pin for this dude
float rc_channel_duty_cycles[NUM_RC_CHANNELS];

#define RC_VAL_SCALING 1000
struct RCVals {
    float left_joystick_x;
    float left_joystick_y;
    float right_joystick_x;
    float right_joystick_y;
};
RCVals rc_vals;

// struct RobotController {
//     int 
// }

#define NUM_INPUTS 3
#define NUM_OUTPUTS 5
#define ENC_COUNTS_PER_ROT 85.0
#define PI 3.141592
#define INTERRUPT_PERIOD 0.0001
#define MAIN_LOOP_PERIOD 0.001
#define EPSILON 0.001

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2
#define MOTOR_D 3

Timer t;                    // Timer to measure elapsed time of experiment
Ticker ControlLoop;       //Ticker object created

QEI encoderA(PE_9,PE_11, NC, ENC_COUNTS_PER_ROT, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, ENC_COUNTS_PER_ROT, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC(PC_6, PC_7, NC, ENC_COUNTS_PER_ROT, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, ENC_COUNTS_PER_ROT, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000);

float rc_duty_cycle_to_joystick_val(float duty_cycle);

void motor_control();
void run_motor(int motor_id, float speed);
void stop_motors();

int main()
{
    pc.printf("Init...\n\r");

    ControlLoop.attach(&motor_control, INTERRUPT_PERIOD); //Run current controller at 10khz
    
    // Setup experiment
    t.reset();
    t.start();
    encoderA.reset();
    encoderB.reset();
    encoderC.reset();
    encoderD.reset();

    stop_motors();

    while (1) {
        rc_vals.left_joystick_x = rc_duty_cycle_to_joystick_val(ch4.dutycycle());
        rc_vals.left_joystick_y = rc_duty_cycle_to_joystick_val(ch1.dutycycle());
        rc_vals.right_joystick_x = rc_duty_cycle_to_joystick_val(ch2.dutycycle());
        rc_vals.right_joystick_y = rc_duty_cycle_to_joystick_val(ch3.dutycycle());
        // pc.printf("%f %f %f %f\n\r", ch1.dutycycle(), ch4.dutycycle(), ch2.dutycycle(), ch3.dutycycle());
        // pc.printf("%.3f %.3f %.3f %.3f --> ", rc_vals.left_joystick_x, rc_vals.left_joystick_y, rc_vals.right_joystick_x, rc_vals.right_joystick_y);

        bool command_north_south = abs(rc_vals.left_joystick_y) > 2 * abs(rc_vals.left_joystick_x);
        bool command_east_west = abs(rc_vals.left_joystick_x) > 2 * abs(rc_vals.left_joystick_y);
        bool command_northwest_southeast = abs(abs(rc_vals.left_joystick_x) - abs(rc_vals.left_joystick_y)) < 0.35 && rc_vals.left_joystick_x != 0.0 && rc_vals.left_joystick_y != 0.0 && rc_vals.left_joystick_y/rc_vals.left_joystick_x < 0.0;
        bool command_northeast_southwest = abs(abs(rc_vals.left_joystick_x) - abs(rc_vals.left_joystick_y)) < 0.35 && rc_vals.left_joystick_x != 0.0 && rc_vals.left_joystick_y != 0.0 && rc_vals.left_joystick_y/rc_vals.left_joystick_x > 0.0;
        bool command_counterclockwise = rc_vals.right_joystick_x < 0.0;
        bool command_clockwise = rc_vals.right_joystick_x > 0.0;

        // pc.printf("NS: %d EW: %d NWSE: %d NESW: %d CC: %d C: %d", command_north_south, command_east_west, command_northwest_southeast, command_northeast_southwest, command_counterclockwise, command_clockwise);
        
        if (command_north_south) {
            float speed = rc_vals.left_joystick_y;
            run_motor(MOTOR_A, -speed);
            run_motor(MOTOR_B, -speed);
            run_motor(MOTOR_C, speed);
            run_motor(MOTOR_D, speed);
            pc.printf("NS: %.3f\r\n", speed);
        } else if (command_east_west) {
            float speed = rc_vals.left_joystick_x;
            run_motor(MOTOR_A, speed);
            run_motor(MOTOR_B, -speed);
            run_motor(MOTOR_C, -speed);
            run_motor(MOTOR_D, speed);
            pc.printf("EW: %.3f\r\n", speed);
        } else if (command_northwest_southeast) {
            int sign = rc_vals.left_joystick_x < 0 ? -1 : 1;
            float speed = sign * sqrt(rc_vals.left_joystick_x*rc_vals.left_joystick_x + rc_vals.left_joystick_y*rc_vals.left_joystick_y)/sqrt(2);
            run_motor(MOTOR_A, speed);
            run_motor(MOTOR_B, 0);
            run_motor(MOTOR_C, -speed);
            run_motor(MOTOR_D, 0);
            pc.printf("NWSE: %.3f\r\n", speed);
        } else if (command_northeast_southwest) {
            int sign = rc_vals.left_joystick_x < 0 ? -1 : 1;
            float speed = sign * sqrt(rc_vals.left_joystick_x*rc_vals.left_joystick_x + rc_vals.left_joystick_y*rc_vals.left_joystick_y)/sqrt(2);
            run_motor(MOTOR_A, 0);
            run_motor(MOTOR_B, -speed);
            run_motor(MOTOR_C, 0);
            run_motor(MOTOR_D, speed);
            pc.printf("NESW: %.3f\r\n", speed);
        } else if (command_counterclockwise) {
            float speed = rc_vals.right_joystick_x;
            run_motor(MOTOR_A, speed);
            run_motor(MOTOR_B, speed);
            run_motor(MOTOR_C, speed);
            run_motor(MOTOR_D, speed);
            pc.printf("CC: %.3f\r\n", speed);
        } else if (command_clockwise) {
            float speed = rc_vals.right_joystick_x;
            run_motor(MOTOR_A, speed);
            run_motor(MOTOR_B, speed);
            run_motor(MOTOR_C, speed);
            run_motor(MOTOR_D, speed);
            pc.printf("C: %.3f\r\n", speed);
        } else {
            stop_motors();
        }

        // pc.printf("\r\n");
    }

    ControlLoop.detach();
    stop_motors();
}

float rc_duty_cycle_to_joystick_val(float duty_cycle) {
    const int SCALED_MID_DC_VAL = 83;
    const float SCALED_DC_AMPLITUDE = 28.0;
    int shifted_dc_val = (duty_cycle * RC_VAL_SCALING) - SCALED_MID_DC_VAL;
    shifted_dc_val = abs(shifted_dc_val) < 3 ? 0 : shifted_dc_val;
    float normalized_dc_val = shifted_dc_val/SCALED_DC_AMPLITUDE;

    return normalized_dc_val;
}

void motor_control() {

}

void stop_motors() {
    motorShield.motorAWrite(0, 0); //turn motor A off
    motorShield.motorBWrite(0, 0); //turn motor A off
    motorShield.motorCWrite(0, 0); //turn motor A off
    motorShield.motorDWrite(0, 0); //turn motor A off
}

void run_motor(int motor_id, float speed) {
    float duty = abs(speed);
    int direction = speed < 0 ? 0 : 1;
    pc.printf("%.3f %d\r\n", duty, direction);
    if (motor_id == MOTOR_A) {
        motorShield.motorAWrite(duty, direction);
    } else if (motor_id == MOTOR_B) {
        motorShield.motorBWrite(duty, direction);
    } else if (motor_id == MOTOR_C) {
        motorShield.motorCWrite(duty, direction);
    } else if (motor_id == MOTOR_D) {
        motorShield.motorDWrite(duty, direction);
    }
}
