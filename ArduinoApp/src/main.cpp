#include <Arduino.h>

enum Directions
{
  Clockwise,
  CounterClockwise
};

class Motor
{

  int stepPin;
  int dirPin;

public:
  Motor(int stepPin, int dirPin)
  {
    this->stepPin = stepPin;
    pinMode(stepPin, OUTPUT);
    this->dirPin = dirPin;
    pinMode(dirPin, OUTPUT);
  }

  void MakeStep(int Delay)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(Delay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(Delay);
  }

  void SetDirection(Directions Direction)
  {
    if (Direction == Clockwise)
      digitalWrite(dirPin, HIGH);

    if (Direction == CounterClockwise)
      digitalWrite(dirPin, LOW);
  }
};

Motor *motor1;
Motor *motor2;
int dela = 1800;
char buffer = 'S';

void setup()
{
  motor1 = new Motor(3, 4);
  motor2 = new Motor(6, 7);

  Serial1.begin(9600);
  Serial.begin(9600);
}

void loop()
{

  if (Serial1.available() > 0){
    buffer = Serial1.read();
    Serial.println(buffer);
  }

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
