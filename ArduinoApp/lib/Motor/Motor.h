#ifndef MOTOR_H_
#define MOTOR_H_

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
    Motor(int stepPin, int dirPin);
    void SetDirection(Directions Direction);
    void MakeStep(int Delay);
};


#endif