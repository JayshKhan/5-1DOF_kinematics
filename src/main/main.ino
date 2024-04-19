#include <Servo.h>

#define SERVO_COUNT 1
#define DELAY 500
#define SERVO_MIN_PWM 500
#define SERVO_MAX_PWM 2500


Servo servos[SERVO_COUNT];

void setup()
{
    Serial.begin(9600);
    servos[0].attach(1,SERVO_MIN_PWM,SERVO_MAX_PWM);
}

void loop()
{
    int angles[SERVO_COUNT] = {90};
    moveTo(servos, angles);
    delay(DELAY);

}

boolean moveTo(Servo servos[], int angles[])
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        // If angle is -1 then the servo should not move
        if(angles[i] ==-1)
        {
            continue;
        }
        servos[i].write(angles[i]);
        delay(DELAY);
    }
    return true;
}
