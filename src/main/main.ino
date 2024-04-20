#include <Servo.h>

#define SERVO_COUNT 1
#define DELAY 500
#define SERVO_MIN_PWM 500
#define SERVO_MAX_PWM 2500

int buttonPin0 = 9;
int buttonPin90 = 10;
int buttonPin180 = 11;
int buttonState0=LOW;
int buttonState90=LOW;
int buttonState180=LOW;

Servo servos[SERVO_COUNT];
int servo_pin[SERVO_COUNT]={3}
int currentAngles[SERVO_COUNT]={0};
int angles[SERVO_COUNT] = {0};

void setup()
{
    Serial.begin(9600);
    servos[0].attach(3,SERVO_MIN_PWM,SERVO_MAX_PWM);
    pinMode(buttonPin0, INPUT_PULLUP);
    pinMode(buttonPin90, INPUT_PULLUP);
    pinMode(buttonPin180, INPUT_PULLUP);
}

void loop()
{
  
   buttonState0 = !digitalRead(buttonPin0);
   buttonState90 = !digitalRead(buttonPin90);
   buttonState180 = !digitalRead(buttonPin180);
  Serial.println(buttonState0==HIGH);
    if (buttonState0 == HIGH) {
    Serial.println("setting angle 0");
    angles[0]=0;
  } 

  if (buttonState90 == HIGH) {
    Serial.println("setting angle 90");
    angles[0]=90;
  } 
  
  if (buttonState180 == HIGH) {
    Serial.println("setting angle 180");
    angles[0]=180;
  } 
    
    moveTo(angles);
    delay(DELAY);

}

boolean moveTo( int angles[])
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        // If angle is -1 then the servo should not move
        if(angles[i] ==-1 || angles[i]==currentAngles[i])
        {
            continue;
        }
        if(currentAngles[i]<angles[i]){
          for(int j = currentAngles[i];j!=angles[i];j+=5){
            if (j+5<angles[i]){
              servos[i].write(j);
            }
            else{
              servos[i].write(angles[i]);
              break;
            }
          }
        }
        else {
          for(int j = currentAngles[i];j!=angles[i];j-=5){
            if (j-5>angles[i]){
              servos[i].write(j);
            }
            else{
              servos[i].write(angles[i]);
              break;
            }
            
          }
        }
        
        currentAngles[i]=angles[i];
        
        delay(DELAY);
    }
    return true;
}
