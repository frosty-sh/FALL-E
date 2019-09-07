#include <Arduino.h>
#include <Motor.h>

Motor *motor1;
Motor *motor2;
int dela = 2500;
char buffer = 'S';

void setup()
{
  motor1 = new Motor(3, 4);
  motor2 = new Motor(6, 7);

  Serial1.begin(9600); //Begin HC-05 serial communication
  Serial.begin(9600);  //Begina serial port for debugging
}

void loop()
{

  //Read HC-05 incoming data
  if (Serial1.available() > 0)
    buffer = Serial1.read();

  if (buffer == 'U')
  {
    Serial.println("Naprijed");
    motor1->SetDirection(Clockwise);
    motor2->SetDirection(CounterClockwise);
    motor1->MakeStep(dela);
    motor2->MakeStep(dela);
  }

  if (buffer == 'D')
  {
    motor1->SetDirection(CounterClockwise);
    motor2->SetDirection(Clockwise);
    motor1->MakeStep(dela);
    motor2->MakeStep(dela);
  }
}