//
// Created by j-ubuntuworkstation on 4/21/24.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Servo.h>

#include <math.h>

#define SERVO_COUNT 6 // number of servos
#define DELAY 500 // delay between each movement
#define SERVO_MIN_PWM 500 // minimum pulse width
#define SERVO_MAX_PWM 2500 // maximum pulse width
#define SPEED 5 // speed of servo movement

// for Fixing the servo at 0, 90, 180
int buttonPin0 = 9;
int buttonPin90 = 10;
int buttonPin180 = 11;
int buttonState0 = LOW;
int buttonState90 = LOW;
int buttonState180 = LOW;
//----------------------------

Servo servos[SERVO_COUNT];
int servo_pin[SERVO_COUNT] = {
    3,4,5,6,7,8
  }; //{3,4,5,6,7,8};
int initialAngles[SERVO_COUNT] = {
    0
  }; //initializing all with Zero
int currentAngles[SERVO_COUNT] = {
    0
  }; //initializing all with Zero
int angles[SERVO_COUNT] = {
    0
  }; //initializing all with Zero

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
