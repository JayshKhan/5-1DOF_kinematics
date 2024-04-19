//
// Created by j-ubuntuworkstation on 4/19/24.
//

#include <iostream>
#def

using namespace std;

int main()
{

}
#define SERVO_COUNT 6;
#define DELAY 500;
#define SERVO_MIN_PWM 500;
#define SERVO_MAX+PWM 2500;


Servo servos[SERVO_COUNT];

void setup()
{
    Serial.begin(9600);
    for (int i = 1; i <= SERVO_COUNT; i++)
    {
        servos[i-1].attach(i, SERVO_MIN_PWM, SERVO_MAX_PWM);
    }
}

void loop()
{
    int angles[6] = {0, 0, 0, 0, 0, 0};
    moveTo(servos, angles);
    delay(DELAY);
    angles[0] = 90;
    moveTo(servos, angles);
    delay(DELAY);
    angles[0] = 0;
    angles[1] = 90;
    moveTo(servos, angles);
    delay(DELAY);
    angles[1] = 0;
    angles[2] = 90;
    moveTo(servos, angles);
    delay(DELAY);
    angles[2] = 0;
    angles[3] = 90;
    moveTo(servos, angles);
    delay(DELAY);
    angles[3] = 0;
    angles[4] = 90;
    moveTo(servos, angles);
    delay(DELAY);
    angles[4] = 0;
    angles[5] = 90;
    moveTo(servos, angles);
    delay(DELAY);
    angles[5] = 0;
    moveTo(servos, angles);
    delay(DELAY);
}

boolean moveTo(int[] servos, int[] angles)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        // If angle is -1 then the servo should not move
        if(angle[i] ==-1)
        {
            continue;
        }
        servos[i].write(angles[i]);
        delay(DELAY);
    }
    return true;
}