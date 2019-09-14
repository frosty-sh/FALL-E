#include <Arduino.h>
#include <Motor.h>
#include <MPU6050.h>

MPU6050 *Mpu6050;
HardwareSerial HC05 = Serial1;
char Buffer = 'S';

Motor MotorLeft;
Motor MotorRight;
int maxSpeed;

void setup()
{
  //Initialize motors
  MotorLeft.connectToPins(50, 51);
  MotorRight.connectToPins(52, 53);
  //Set speed
  MotorLeft.setSpeed(3500);
  MotorRight.setSpeed(3500);
  //Set acceleration
  MotorLeft.setAcceleration(17500);
  MotorRight.setAcceleration(17500);

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
      MotorRight.setupMove(CounterClockwise);
      MotorLeft.setupMove(CounterClockwise);
      break;
    }

    case 'U':
    {
      MotorRight.setupMove(CounterClockwise);
      MotorLeft.setupMove(Clockwise);
      break;
    }

    case 'D':
    {
      MotorRight.setupMove(Clockwise);
      MotorLeft.setupMove(CounterClockwise);
      break;
    }

    case 'L':
    {
      MotorRight.setupMove(Clockwise);
      MotorLeft.setupMove(Clockwise);
      break;
    }

    default:
      break;
    }

    while ((!MotorLeft.motionComplete()) || (!MotorRight.motionComplete()))
    {
      //Make step
      MotorLeft.processMovement();
      MotorRight.processMovement();

      //Read serial for stop signal
      if (Serial1.available() > 0 && Serial1.read() == 'S')
      {
        //Start decelerating motors
        MotorLeft.setupStop();
        MotorRight.setupStop();

        Buffer = 'S';
        continue;
      }
    }

    //Clear buffer
    while (Serial1.available() > 0)
      Serial1.read();
  }
}