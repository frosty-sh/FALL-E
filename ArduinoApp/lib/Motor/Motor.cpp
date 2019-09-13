#include <Motor.h>
#include <Arduino.h>

Motor::Motor(int stepPin, int dirPin)
{
  this->stepPin = stepPin;
  pinMode(stepPin, OUTPUT);
  this->dirPin = dirPin;
  pinMode(dirPin, OUTPUT);
}

void Motor::MakeStep(int Delay)
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(Delay);
  digitalWrite(stepPin, LOW);
}

void Motor::SetDirection(Directions Direction)
{
  if (Direction == Clockwise)
    digitalWrite(dirPin, HIGH);

  if (Direction == CounterClockwise)
    digitalWrite(dirPin, LOW);
}