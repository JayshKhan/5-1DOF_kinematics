
/*
 * This code is for controlling the servo motors of ROT3U 5+1 DOF Robotic Arm using the Arduino board.
 * The code is written in C++ and uses the Arduino library for controlling the servo motors.
 * @author Jaysh Khan
 */

#include "./forwardkinematics.h"

void setup() {
  Serial.begin(9600);  //initialize serial communication
  // Initialize the PID controller for the servos



  for (int i = 0; i < SERVO_COUNT; i++) {
    pids[i]= new PID(&inputs[i], &outputs[i], &angles[i], Kp, Ki, Kd, P_ON_E, DIRECT);
    pids[i]->SetMode(AUTOMATIC);
    pids[i]->SetSampleTime(100);       // Set PID sample time to 100 milliseconds
    pids[i]->SetOutputLimits(0, 180);  // Set PID output limits (0-180 degrees)

    if (PWMDriver) {
      servosP[i].begin();
      servosP[i].setOscillatorFrequency(27000000);
      servosP[i].setPWMFreq(SERVO_FREQ);
    } else {
      servos[i].attach(servo_pin[i], SERVO_MIN_PWM, SERVO_MAX_PWM);
      Serial.println("Attaching :"+String(i+1)+" to "+String(servo_pin[i]));
    }
  }
}

void loop() {
  keyBoardControl();
  // int angle[1]={0};
  moveServosSequentially();
  //  float alpha[SERVO_COUNT-1] = {0, 90, 0, 0, 90};
  //   int theta[SERVO_COUNT-1] = {0, 0, 0, 0, 0};
  //   float a[SERVO_COUNT-1] = {0, 0, 3, 4, 3};
  //   float d[SERVO_COUNT-1] = {0, 0, 0, 0, 0};
  //   Matrix4x4 * T = T_matrices(alpha, theta, a, d);
  //   Matrix4x4 T_0_5 = forwardKinematics(T);
  //   float x, y, z;
  //   T_0_5.getXYZ(&x, &y, &z);
  //   Serial.println("0X: " + String(x) + " Y: " + String(y) + " Z: " + String(z));
  // //------------------------------------------
  //   // for Fixing the servo at 0, 90, 180
}

/**
 * Move the servos to the specified angles one servo at a time
 * @param angles array of angles for each servo
 * @author Jaysh Khan
 */

void moveServosSequentially() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    //PID control
    inputs[i] = currentAngles[i];
    pids[i]->Compute();
    angles[i] = outputs[i];
    Serial.println("output: "+String(outputs[i])+" angles: "+String(angles[i])+"");

    // If angle is -1 then the servo should not move
    if (angles[i] == -1 || angles[i] == currentAngles[i]) {
      continue;
    }
    int direction = angles[i] > currentAngles[i] ? 1 : -1;
    for (int j = currentAngles[i]; j != angles[i]; j += direction * SPEED) {
   
      if (direction == -1 && j - SPEED > angles[i]) {
        if (PWMDriver) {
          servosP[i].setPWM(servo_pin[i], 0, angleToPulse(j));
          // servosP[i].writeMicroseconds(servo_pin[i],angleToMS(j));
        } else {
          Serial.println("Moving to: "+String(j));
          servos[i].write(j);
        }
      } else if (direction == 1 && j + SPEED < angles[i]) {
        if (PWMDriver) {
          servosP[i].setPWM(servo_pin[i], 0, angleToPulse(j));
          // servosP[i].writeMicroseconds(servo_pin[i],angleToMS(j));
        } else {
          Serial.println("Moving to: "+String(j));
          servos[i].write(j);

        }
      } else {
        if (PWMDriver) {
          servosP[i].setPWM(servo_pin[i], 0, angleToPulse(angles[i]));
          // servos[i].writeMicroseconds(servo_pin[i],angleToMS(angles[i]));
        } else {
          Serial.println("Moving to: "+String(angles[i]));
          servos[i].write(angles[i]);
        }


        break;
      }
      // delay(DELAY);
    }
    //     if (currentAngles[i] < angles[i]) {
    //       // smoothly moving to that angle
    //       for (int j = currentAngles[i]; j != angles[i]; j += SPEED) {
    //         if (j + SPEED < angles[i]) {
    //           servos[i].write(j);
    //         } else {
    //           servos[i].write(angles[i]);
    //           break;
    //         }
    //       }
    //     } else {
    //       for (int j = currentAngles[i]; j != angles[i]; j -= SPEED) {
    //         if (j - SPEED > angles[i]) {
    //           servos[i].write(j);
    //         } else {
    //           servos[i].write(angles[i]);
    //           break;
    //         }
    //
    //       }
    //     }

    currentAngles[i] = angles[i];

    delay(DELAY);
  }
}

/**
 * Move the servos to the specified angles simultaneously
 * @param angles array of angles for each servo
 * @author Jaysh Khan
 * work in progress...
 */

// void moveServosSimultaneously(int angles[]) {
//   // Flag to track if any servo needs movement
//   bool needsUpdate = false;

//   // Loop through each servo
//   for (int i = 0; i < SERVO_COUNT; i++) {
//     // Check if angle needs update (not -1 or different from current)
//     if (angles[i] != -1 && angles[i] != currentAngles[i]) {
//       needsUpdate = true;
//     }
//   }

//   // If any servo needs update, move them simultaneously with step-by-step approach
//   if (needsUpdate) {
//     for (int i = 0; i < SERVO_COUNT; i++) {
//       int direction = (angles[i] > currentAngles[i]) ? 1 : -1;
//       // Move towards target angle with SPEED steps
//       if (currentAngles[i] + direction * SPEED < angles[i]) {
//         currentAngles[i] += direction * SPEED;
//       } else {
//         currentAngles[i] = angles[i];
//       }
//       servos[i].write(currentAngles[i]);
//     }
//   }

//   delay(DELAY); // Add a delay after updates
// }

/**
 * Reset the servo to initial position
 */

void reset() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    angles[i] = initialAngles[i];
  }
  moveServosSequentially();
}



int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVO_MIN_PWM, SERVO_MAX_PWM);  // map angle of 0 to 180 to Servo min and Servo max
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" pulse: ");
  Serial.println(pulse);
  return pulse;
}
int angleToMS(int ang) {
  int ms = map(ang, 0, 180, USMIN, USMAX);  // map angle of 0 to 180 to Servo min and Servo max
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" MS: ");
  Serial.println(ms);
  return ms;
}

void keyBoardControl() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');  // Read string from serial until newline character

    // Split the string into individual angle values
    String anglesArray[SERVO_COUNT];
    int index = 0;
    int startPos = 0;
    for (int i = 0; i < inputString.length(); i++) {
      if (inputString.charAt(i) == ',' || inputString.charAt(i) == '\n') {
        anglesArray[index++] = inputString.substring(startPos, i);
        startPos = i + 1;
      }
    }

    // Check if the correct number of angles is received
    if (index != SERVO_COUNT) {
      Serial.println("Invalid number of angles."+String(index)+" Please provide angles for all servos. "+String(SERVO_COUNT));
      return;
    }

    // Convert each angle to integer and set the angles for each servo
    for (int i = 0; i < SERVO_COUNT; i++) {
      int newAngle = anglesArray[i].toDouble();
      if (newAngle >= 0 && newAngle <= 180) {
        angles[i] = newAngle;
        Serial.println("New angles for servo " + String(i) + ": " + String(angles[i]));
      } else {
        Serial.println("Invalid angle for servo " + String(i) + ". Enter a number between 0 and 180.");
      }
    }
  }
}
