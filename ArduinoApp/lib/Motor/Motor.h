#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

enum Direction{
  Clockwise,
  CounterClockwise
};

class Motor
{
  public:
    Motor();
    void connectToPins(byte stepPinNumber, byte directionPinNumber);

    void setupStop();
    void setSpeed(float speed);
    void setAcceleration(float acceleration);
    void setupMove(Direction direction);
    bool motionComplete();
    void processMovement(void);
    void makeStep ();
    void setDirection(Direction Direction);
  private:

    byte stepPin;
    byte directionPin;
    float desiredSpeed_InStepsPerSecond;
    float acceleration_InStepsPerSecondPerSecond;
    long targetPosition_InSteps;
    bool startNewMove;
    float desiredStepPeriod_InUS;
    long decelerationDistance_InSteps;
    bool stopping;
    bool started;

    float ramp_InitialStepPeriod_InUS;
    float ramp_NextStepPeriod_InUS;
    unsigned long ramp_LastStepTime_InUS;

    float acceleration_InStepsPerUSPerUS;
    float currentStepPeriod_InUS;
    
};

#endif