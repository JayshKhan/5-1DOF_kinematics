/*
*   This code is for controlling 6 servo motors using Servo.h.
*   The code reads the angles for each servo from the serial monitor and moves the servo to that angle.
*   The angles should be separated by commas and should be in the range of 0 to 180.
*   The code moves the servo to the target angle with a speed of 5 degrees per step.
*   @author: Jaysh Khan
*/

#include <Servo.h>

#define SERVO_COUNT 6
#define DELAY 500
#define SPEED 5

Servo servos[SERVO_COUNT] = {};
int Min_PWM[SERVO_COUNT]= {500};
int Max_PWM[SERVO_COUNT]= {2500};


int servo_pin[SERVO_COUNT] = {
    2,3,4,5,6,7
  }; //{3,4,5,6,7,8};
double initialAngles[SERVO_COUNT] = {
    0.0,0.0,0.0,0.0,0.0,0.0
  }; //initializing all with Zero
double currentAngles[SERVO_COUNT] = {
    0.0,0.0,0.0,0.0,0.0,0.0
  }; //initializing all with Zero
double targetAngles[SERVO_COUNT] = {
    0,0,0,0,0,0
  }; //initializing all with Zero


void setup() {
  Serial.begin(9600); // Initialize serial communication

  for(int i=0;i<SERVO_COUNT;i++)
  {
    servos[i].attach(servo_pin[i]);
  }
}

void loop() {
  pySerialControl(); // Read angles from serial monitor sent by Python script
  moveServosSequentially(); // Move servos to target angles
}

void moveServosSequentially() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (targetAngles[i] == -1 || targetAngles[i] == currentAngles[i]) {
      continue;
    }
    int direction = targetAngles[i] > currentAngles[i] ? 1 : -1;
    for (int j = currentAngles[i]; j != targetAngles[i]; j += direction * SPEED) {

      if (direction == -1 && j - SPEED > targetAngles[i]) {
          servos[i].write(j);

      } else if (direction == 1 && j + SPEED < targetAngles[i]) {
          servos[i].write(j);
      } else {
          servos[i].write(targetAngles[i]);
        break;
      }
       delay(30);
    }
    currentAngles[i] = targetAngles[i];
    delay(DELAY);
  }
}



void pySerialControl() {
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
        targetAngles[i] = newAngle;
        Serial.println("New angles for servo " + String(i) + ": " + String(targetAngles[i]));
      } else {
        Serial.println("Invalid angle for servo " + String(i) + ". Enter a number between 0 and 180.");
      }
    }
  }
}



