
#include <Adafruit_PWMServoDriver.h>

#define SERVO_COUNT 6
#define DELAY 500
#define SPEED 5

Adafruit_PWMServoDriver servos[SERVO_COUNT] = {Adafruit_PWMServoDriver()};
int Min_PWM[SERVO_COUNT]= {125,110,125,100,125,125};
int Max_PWM[SERVO_COUNT]= {600,570,575,555,575,575};


int servo_pin[SERVO_COUNT] = {
    1,2,3,4,5,6
  }; //{3,4,5,6,7,8};
double initialAngles[SERVO_COUNT] = {
    0.0,0.0,0.0,0.0,0.0,0.0
  }; //initializing all with Zero
double currentAngles[SERVO_COUNT] = {
    0.0,0.0,0.0,0.0,0.0,0.0
  }; //initializing all with Zero
double targetAngles[SERVO_COUNT] = {
    10,10,10,10,10,10
  }; //initializing all with Zero


void setup() {
  Serial.begin(9600);

  for(int i=0;i<SERVO_COUNT;i++)
  {
    servos[i].begin();
    servos[i].setOscillatorFrequency(27000000);
    servos[i].setPWMFreq(60);
  }
    // moveServosSequentially();
}

void loop() {
  keyBoardControl();
  moveServosSequentially();
}
void moveServosSequentially() {
  for (int i = 0; i < SERVO_COUNT; i++) {
  // for (int i = SERVO_COUNT; i!=0; i--) {
    // If angle is -1 then the servo should not moveSERVO_COUNT
    if (targetAngles[i] == -1 || targetAngles[i] == currentAngles[i]) {
      continue;
    }
    if(targetAngles[2]>150)
    {
      Serial.println("Moving to maximum it can Go");
      targetAngles[2]=150;
    }
    int direction = targetAngles[i] > currentAngles[i] ? 1 : -1;
    for (int j = currentAngles[i]; j != targetAngles[i]; j += direction * SPEED) {
   
      if (direction == -1 && j - SPEED > targetAngles[i]) {
        
          servos[i].setPWM(servo_pin[i], 0, angleToPulse(j,i));
    
      } else if (direction == 1 && j + SPEED < targetAngles[i]) {
        
          servos[i].setPWM(servo_pin[i], 0, angleToPulse(j,i));
   
      } else {
      
          servos[i].setPWM(servo_pin[i], 0, angleToPulse(targetAngles[i],i));
       
        break;
      }
       delay(30);
    }
    currentAngles[i] = targetAngles[i];
    delay(DELAY);
  }
}

int angleToPulse(int ang,int i) {
  int pulse = map(ang, 0, 180, Min_PWM[i], Max_PWM[i]);  // map angle of 0 to 180 to Servo min and Servo max
  Serial.print("Angle: ");
  Serial.print(ang);
  Serial.print(" pulse: ");
  Serial.println(pulse);
  return pulse;
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
        targetAngles[i] = newAngle;
        Serial.println("New angles for servo " + String(i) + ": " + String(targetAngles[i]));
      } else {
        Serial.println("Invalid angle for servo " + String(i) + ". Enter a number between 0 and 180.");
      }
    }
  }
}



