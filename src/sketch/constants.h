//
// Created by Jaysh Khan on 4/21/24.
// @author Jaysh Khan
//

#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Servo.h>
#include<Wire.h>
#include<PID_v1.h>
#include <Adafruit_PWMServoDriver.h>

#include <math.h>
const bool PWMDriver= false; // 0 if Servo.h
#define SERVO_COUNT 6 // number of servos
#define DELAY 500 // delay between each movement
#define SERVO_MIN_PWM 500 // minimum pulse width
#define SERVO_MAX_PWM 2500 // maximum pulse width
#define SPEED 3 // speed of servo movement
#define SERVO_FREQ 60

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600


Adafruit_PWMServoDriver servosP[SERVO_COUNT]={Adafruit_PWMServoDriver()};
Servo servos[SERVO_COUNT];



int servo_pin[SERVO_COUNT] = {
    3,5,6,9,10,11
  }; //{3,4,5,6,7,8};
double initialAngles[SERVO_COUNT] = {
    0.0,0.0,0.0,0.0,0.0,0.0
  }; //initializing all with Zero
double currentAngles[SERVO_COUNT] = {
    0.0,0.0,0.0,0.0,0.0,0.0
  }; //initializing all with Zero
double angles[SERVO_COUNT] = {
    0.0,0.0,0.0,0.0,0.0,0.0
  }; //initializing all with Zero

double inputs[SERVO_COUNT], outputs[SERVO_COUNT];
double errors[SERVO_COUNT], integral[SERVO_COUNT], derivative[SERVO_COUNT];
double Kp = 1.0, Ki = 1, Kd = 0.5;  // Adjust PID constants

// Create PID controller objects for each servo
PID* pids[SERVO_COUNT];


/**
* Kinematics Stuff
*/


struct Matrix4x4 {
    String name = "temp"; // T_0_1 , T_1_2, T_2_3, T_3_4, T_4_5, T_5_6
    float data[4][4];
    void getXYZ(float * x, float * y, float * z) {
        * x = data[0][3];
        * y = data[1][3];
        * z = data[2][3];
    }
    // initialize the matrix with identity matrix
    Matrix4x4() {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                if (i == j) {
                    data[i][j] = 1;
                } else {
                    data[i][j] = 0;
                }
            }
        }
    }
};

Matrix4x4 forwardKinematics(Matrix4x4);
Matrix4x4 * T_matrices(float[], int[], float[], float[]);
Matrix4x4 Multiply(Matrix4x4, Matrix4x4);

#endif //CONSTANTS_H
