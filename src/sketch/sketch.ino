
/*
 * This code is for controlling the servo motors of ROT3U 5+1 DOF Robotic Arm using the Arduino board.
 * The code is written in C++ and uses the Arduino library for controlling the servo motors.
 * @author Jaysh Khan
 */

#include "./forwardkinematics.h"

void setup() {
  Serial.begin(9600); //initialize serial communication
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(servo_pin[i], SERVO_MIN_PWM, SERVO_MAX_PWM);
  }
  // for Fixing the servo at 0, 90, 180
  pinMode(buttonPin0, INPUT_PULLUP);
  pinMode(buttonPin90, INPUT_PULLUP);
  pinMode(buttonPin180, INPUT_PULLUP);
}

void loop() {
// testing the forward kinematics
  float alpha[SERVO_COUNT-1] = {0, 90, 0, 0, 90};
  int theta[SERVO_COUNT-1] = {0, 0, 0, 0, 0};
  float a[SERVO_COUNT-1] = {0, 0, 3, 4, 3};
  float d[SERVO_COUNT-1] = {0, 0, 0, 0, 0};
  Matrix4x4 * T = T_matrices(alpha, theta, a, d);
  Matrix4x4 T_0_5 = forwardKinematics(T);
  float x, y, z;
  T_0_5.getXYZ(&x, &y, &z);
  Serial.println("0X: " + String(x) + " Y: " + String(y) + " Z: " + String(z));
//------------------------------------------
  // for Fixing the servo at 0, 90, 180
  buttonState0 = !digitalRead(buttonPin0);
  buttonState90 = !digitalRead(buttonPin90);
  buttonState180 = !digitalRead(buttonPin180);
  Serial.println(buttonState0 == HIGH);
  if (buttonState0 == HIGH) {
    Serial.println("setting angle 0");
    angles[0] = 0;
  }

  if (buttonState90 == HIGH) {
    Serial.println("setting angle 90");
    angles[0] = 90;
  }

  if (buttonState180 == HIGH) {
    Serial.println("setting angle 180");
    angles[0] = 180;
  }

  moveTo(angles);
  delay(DELAY);

}

/**
 * Move the servos to the specified angles
 * @param angles array of angles for each servo
 * @author Jaysh Khan
 */

void moveTo(int angles[]) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    // If angle is -1 then the servo should not move
    if (angles[i] == -1 || angles[i] == currentAngles[i]) {
      continue;
    }
    if (currentAngles[i] < angles[i]) {
      // smoothly moving to that angle
      for (int j = currentAngles[i]; j != angles[i]; j += SPEED) {
        if (j + SPEED < angles[i]) {
          servos[i].write(j);
        } else {
          servos[i].write(angles[i]);
          break;
        }
      }
    } else {
      for (int j = currentAngles[i]; j != angles[i]; j -= SPEED) {
        if (j - SPEED > angles[i]) {
          servos[i].write(j);
        } else {
          servos[i].write(angles[i]);
          break;
        }

      }
    }

    currentAngles[i] = angles[i];

    delay(DELAY);
  }
}

/**
 * Reset the servo to initial position
 */

void reset() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    angles[i] = initialAngles[i];
  }
  moveTo(angles);
}
