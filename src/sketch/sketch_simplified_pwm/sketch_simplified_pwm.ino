/*
*   This code is for controlling 6 servo motors using PCA9685 servo driver.
*   The code reads the angles for each servo from the serial monitor and moves the servo to that angle.
*   The angles should be separated by commas and should be in the range of 0 to 180.
*   The code moves the servo to the target angle with a speed of 5 degrees per step.
*   @author: Jaysh Khan
*/
#include <Adafruit_PWMServoDriver.h>

#define SERVO_COUNT 6
#define DELAY 500
#define SPEED 5
char Mode = 's';

Adafruit_PWMServoDriver servos[SERVO_COUNT] = { Adafruit_PWMServoDriver() };
int Min_PWM[SERVO_COUNT] = { 125, 110, 125, 125, 130, 125 };
int Max_PWM[SERVO_COUNT] = { 600, 570, 575, 575, 575, 575 };


int servo_pin[SERVO_COUNT] = {
  1, 2, 3, 4, 5, 6
};  //{3,4,5,6,7,8};
double initialAngles[SERVO_COUNT] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};  //initializing all with Zero
double currentAngles[SERVO_COUNT] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};  //initializing all with Zero
double targetAngles[SERVO_COUNT] = {
  10, 10, 10, 10, 10, 10
};  //initializing all with Zero


void setup() {
  Serial.begin(9600);  // Initialize serial communication

  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].begin();                           // Initialize the servo driver
    servos[i].setOscillatorFrequency(27000000);  // The int.oscillator frequency is 27MHz
    servos[i].setPWMFreq(60);                    // Analog servos run at ~60 Hz updates
  }
}

void loop() {
  pySerialControl();  // Read angles from serial monitor sent by Python script
  if (Mode == 's') {
    moveServosSequentially();  // Move servos to target angles
  } else if (Mode == 'c') {
    moveServosSimultaneously();  // Move servos to target angles
  }
}

/*
*   the following function moves one servo at a time
*   to the target angle with a some speed of SPEED degrees per step.
*   @param: None
*   @return: None
*/
void moveServosSequentially() {

  for (int i = 0; i < SERVO_COUNT; i++) {
    if (targetAngles[i] == -1 || targetAngles[i] == currentAngles[i]) {
      continue;
    }
    int direction = targetAngles[i] > currentAngles[i] ? 1 : -1;
    for (int j = currentAngles[i]; j != targetAngles[i]; j += direction * SPEED) {

      if (direction == -1 && j - SPEED > targetAngles[i]) {

        servos[i].setPWM(servo_pin[i], 0, angleToPulse(j, i));

      } else if (direction == 1 && j + SPEED < targetAngles[i]) {

        servos[i].setPWM(servo_pin[i], 0, angleToPulse(j, i));

      } else {

        servos[i].setPWM(servo_pin[i], 0, angleToPulse(targetAngles[i], i));

        break;
      }
      delay(30);
    }
    currentAngles[i] = targetAngles[i];
    delay(DELAY);
  }
}
void moveServosSimultaneously() {
  Serial.println("Moving Simultaneously");

  // Calculate the maximum number of steps needed for any servo
  int maxSteps = 0;
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (targetAngles[i] != -1 && targetAngles[i] != currentAngles[i]) {
      int steps = abs(targetAngles[i] - currentAngles[i]) / SPEED;
      if (steps > maxSteps) {
        maxSteps = steps;
      }
    }
  }

  // Move all servos in steps until they reach their target angles
  for (int step = 0; step <= maxSteps; step++) {
    for (int i = 0; i < SERVO_COUNT; i++) {
      if (targetAngles[i] == -1 || targetAngles[i] == currentAngles[i]) {
        continue; // Skip servos that don't need to move
      }

      int direction = targetAngles[i] > currentAngles[i] ? 1 : -1;
      int nextAngle = currentAngles[i] + direction * SPEED * step;

      // Ensure the servo doesn't overshoot the target angle
      if ((direction == 1 && nextAngle > targetAngles[i]) || 
          (direction == -1 && nextAngle < targetAngles[i])) {
        nextAngle = targetAngles[i];
      }

      servos[i].setPWM(servo_pin[i], 0, angleToPulse(nextAngle, i));
      currentAngles[i] = nextAngle;
    }
    delay(100);
  }

  // Ensure all servos have reached their final target angle
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (targetAngles[i] != -1 && targetAngles[i] != currentAngles[i]) {
      servos[i].setPWM(servo_pin[i], 0, angleToPulse(targetAngles[i], i));
      currentAngles[i] = targetAngles[i];
    }
  }
  delay(DELAY);
}

int angleToPulse(int ang, int i) {
  int pulse = map(ang, 0, 180, Min_PWM[i], Max_PWM[i]);  // map angle of 0 to 180 to Servo min and Servo max
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" pulse: ");
  Serial.println(pulse);
  return pulse;
}


void pySerialControl() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');  // Read string from serial until newline character

    // Split the string into individual angle values
    String anglesArray[SERVO_COUNT];
    char mode;
    int index = 0;
    int startPos = 0;
    for (int i = 0; i < inputString.length(); i++) {
      if (inputString.charAt(i) == ',' || inputString.charAt(i) == '\n') {
        if (index >= SERVO_COUNT) {
          break;
        }
        anglesArray[index++] = inputString.substring(startPos, i);
        startPos = i + 1;
      }
    }
    for (int i = 0; i < inputString.length(); i++) {
      if (inputString.charAt(i) == 's' || inputString.charAt(i) == 'c') {
        mode = inputString.charAt(i);
        break;
      }
    }

    // Check if the correct number of angles is received
    if (index != SERVO_COUNT) {
      Serial.println("Invalid number of angles." + String(index) + " Please provide angles for all servos. " + String(SERVO_COUNT));
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

    // Process mode character
    if (mode == 's') {
      Serial.println("Sequential mode");
      Mode = mode;
    } else if (mode == 'c') {
      Serial.println("Simultaneous mode");
      Mode = mode;
    } else {
      Serial.println(String(mode) + " Mode");
      Serial.println("Invalid mode character. Use 's' for Sequential or 'c' for Simultaneous mode.");
      return;
    }
  }
}
