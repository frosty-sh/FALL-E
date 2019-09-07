#include <Arduino.h>
#include <Motor.h>
#include <MPU6050.h>

Motor *MotorRight;
Motor *MotorLeft;
MPU6050* Mpu6050;

int dela = 2500;
char buffer = 'S';

void setup()
{
  //Initialize motors
  MotorRight = new Motor(3, 4);
  MotorLeft = new Motor(6, 7);

  Mpu6050= new MPU6050(0x68);//Initialize Gyro-accelerometer

  Serial1.begin(9600); //Begin HC-05 serial communication
  Serial.begin(9600);  //Begina serial port for debugging
}

void loop()
{

  //Read HC-05 incoming data
  if (Serial1.available() > 0)
    buffer = Serial1.read();

  //Move forward
  if (buffer == 'U')
  {
    MotorRight->SetDirection(Clockwise);
    MotorLeft->SetDirection(CounterClockwise);
    MotorRight->MakeStep(dela);
    MotorLeft->MakeStep(dela);
  }

  //Move backwards
  if (buffer == 'D')
  {
    MotorRight->SetDirection(CounterClockwise);
    MotorLeft->SetDirection(Clockwise);
    MotorRight->MakeStep(dela);
    MotorLeft->MakeStep(dela);
  }
}