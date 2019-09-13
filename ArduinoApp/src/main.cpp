#include <Arduino.h>
#include <Motor.h>
#include <MPU6050.h>
#include <SpeedyStepper.h>

MPU6050 *Mpu6050;
HardwareSerial HC05 = Serial1;
char Buffer = 'S';

SpeedyStepper MotorLeft;
SpeedyStepper MotorRight;
int maxSpeed;

void MoveBoth()
{
  while ((!MotorLeft.motionComplete()) || (!MotorRight.motionComplete()))
  {
    MotorLeft.processMovement();
    MotorRight.processMovement();
  }
}

void setup()
{
  //Initialize motors
  MotorLeft.connectToPins(50, 51);
  MotorRight.connectToPins(52, 53);
  //Set speed
  maxSpeed = 2800;
  MotorLeft.setSpeedInStepsPerSecond(maxSpeed);
  MotorRight.setSpeedInStepsPerSecond(maxSpeed);
  //Set acceleration
  MotorLeft.setAccelerationInStepsPerSecondPerSecond(17500);
  MotorRight.setAccelerationInStepsPerSecondPerSecond(17500);

  //Initialize Gyro-accelerometer
  Mpu6050 = new MPU6050(0x68);

  //Begin HC-05 serial communication
  Serial1.begin(9600);

  //Begina serial port for debugging
  Serial.begin(9600);
}

void loop()
{
  //Read HC-05 incoming data
  if (Serial1.available() > 0)
    Buffer = Serial1.read();

  //If buffer has info about direction
  if (String("URDL").indexOf(Buffer) >= 0)
  {
    //Set motor directions
    Serial.println(Buffer);
    switch (Buffer)
    {
    case 'R':
    {
      MotorRight.setupRelativeMoveInSteps(-maxSpeed);
      MotorLeft.setupRelativeMoveInSteps(-maxSpeed);
      break;
    }

    case 'U':
    {
      MotorRight.setupRelativeMoveInSteps(-maxSpeed);
      MotorLeft.setupRelativeMoveInSteps(maxSpeed);
      break;
    }

    case 'D':
    {
      MotorRight.setupRelativeMoveInSteps(maxSpeed);
      MotorLeft.setupRelativeMoveInSteps(-maxSpeed);
      break;
    }

    case 'L':
    {
      MotorRight.setupRelativeMoveInSteps(maxSpeed);
      MotorLeft.setupRelativeMoveInSteps(maxSpeed);
      break;
    }

    default:
      break;
    }

    bool IsStopped = false;

    while ((!MotorLeft.motionComplete()) || (!MotorRight.motionComplete()))
    {
      //Make step
      MotorLeft.processMovement();
      MotorRight.processMovement();

      //Keep moving if the mottor is stopping or hasn't reached max velocity
      if (IsStopped || (abs(MotorLeft.getCurrentVelocityInStepsPerSecond()) != maxSpeed && abs(MotorRight.getCurrentVelocityInStepsPerSecond() != abs(maxSpeed))))
        continue;

      //Read serial for stop signal
      if (Serial1.available() > 0 && Serial1.read() == 'S')
      {
        IsStopped = true;

        //Start decelerating motors
        MotorLeft.setupStop();
        MotorRight.setupStop();

        Buffer = 'S';
        continue;
      }

      //Keep moving motors while no stop signal is recieved
      MotorRight.setCurrentPositionInSteps(0);
      MotorLeft.setCurrentPositionInSteps(0);
    }

    //Clear buffer
    while (Serial1.available() > 0)
      Serial1.read();
  }
}