#include "Motor.h"

// constructor for the stepper class
Motor::Motor()
{
  // initialize constants
  stepPin = 0;
  directionPin = 0;
  currentStepPeriod_InUS = 0.0;
}


// connect the stepper object to the IO pins
void Motor::connectToPins(byte stepPinNumber, byte directionPinNumber)
{
  // remember the pin numbers
  stepPin = stepPinNumber;
  directionPin = directionPinNumber;
  
  // configure the IO bits
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);

  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, LOW);
}


// begin the process of decelerating
void Motor::setupStop()
{
  float currentStepPeriod_InSteps;

  // reverse the acceleration direction to start slowing down
  acceleration_InStepsPerUSPerUS = -acceleration_InStepsPerUSPerUS;

  // determine the number of steps needed to go from the current velocity down to a velocity of 0, Steps = Velocity^2 / (2 * Accelleration)
  currentStepPeriod_InSteps = 1000000.0 / currentStepPeriod_InUS;
  decelerationDistance_InSteps = (long) round((currentStepPeriod_InSteps * currentStepPeriod_InSteps) / (2.0 * abs(acceleration_InStepsPerSecondPerSecond)));

  // set state of motor to stopping
  stopping = true;
}


// set the maximum speed, units in steps/second, this is the maximum speed reached while accelerating
void Motor::setSpeed(float speed)
{
  desiredSpeed_InStepsPerSecond = speed;
}


// set the rate of acceleration, units in steps/second/second
void Motor::setAcceleration(float acceleration)
{
    acceleration_InStepsPerSecondPerSecond = acceleration;
}

// start moving the motor in desired direction
void Motor::setupMove(Direction Direction)
{

  // determine the period in US of the first step
  ramp_InitialStepPeriod_InUS =  1000000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);
    
  // determine the period in US between steps when going at the desired velocity
  desiredStepPeriod_InUS = 1000000.0 / desiredSpeed_InStepsPerSecond;

  // set decelaration in steps to minimum
  decelerationDistance_InSteps = 1;
  
  // set the direction pin state
  if(Direction == Clockwise)
    digitalWrite(directionPin, LOW);
  else
    digitalWrite(directionPin, HIGH);

  // start the acceleration ramp at the beginning
  ramp_NextStepPeriod_InUS = ramp_InitialStepPeriod_InUS;
  acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;
  startNewMove = true;
  stopping = false;
}


// if it is time, move one step
void Motor::processMovement(void)
{ 
  unsigned long currentTime_InUS;
  unsigned long periodSinceLastStep_InUS;

  // check if this is the first call to start this new move
  if (startNewMove)
  {    
    ramp_LastStepTime_InUS = micros();
    startNewMove = false;
  }
    
  // determine how much time has elapsed since the last step
  currentTime_InUS = micros();
  periodSinceLastStep_InUS = currentTime_InUS - ramp_LastStepTime_InUS;

  // if it is not time for the next step, return
  if (periodSinceLastStep_InUS < (unsigned long) ramp_NextStepPeriod_InUS)
    return;

  // if the stepper is in stopping state decrease the distance needed to decelerato to 0
  if (stopping)
    decelerationDistance_InSteps--;
  
  // execute the step on the rising edge
  digitalWrite(stepPin, HIGH);
  
  // delay set to almost nothing because there is so much code between rising and falling edges
  delayMicroseconds(3);        
  
  // update the current speed
  currentStepPeriod_InUS = ramp_NextStepPeriod_InUS;

  // compute the period for the next step
  // StepPeriodInUS = LastStepPeriodInUS * (1 - AccelerationInStepsPerUSPerUS * LastStepPeriodInUS^2)
  ramp_NextStepPeriod_InUS = ramp_NextStepPeriod_InUS * (1.0 - acceleration_InStepsPerUSPerUS * ramp_NextStepPeriod_InUS * ramp_NextStepPeriod_InUS);

  // return the step line high
  digitalWrite(stepPin, LOW);
 
  // clip the speed so that it does not accelerate beyond the desired velocity
  if (ramp_NextStepPeriod_InUS < desiredStepPeriod_InUS)
    ramp_NextStepPeriod_InUS = desiredStepPeriod_InUS;

  // update the acceleration ramp
  ramp_LastStepTime_InUS = currentTime_InUS;
}


// check if the motor has competed its move to the target position
bool Motor::motionComplete()
{
  return(decelerationDistance_InSteps <= 0);
}