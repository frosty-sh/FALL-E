// #include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Motor.h>
#include <TimerOne.h>
#include <Metro.h>

HardwareSerial HC05 = Serial1;
char Buffer = 'S';

Motor MotorLeft;
Motor MotorRight;
int maxSpeed = 300;
int value = 32000;

VectorFloat gravity;
float ypr[3];

//za PID
//varijable za PID
// double Kp = 400, Ki = 80, Kd = 0;

float input, setpoint, error, output, outputConstraint, deadband = 5;
float Kp = 28;
float Ki = 3, accumulatedError, iConstraint = 400;
float Kd = 0.8, lastError;

MPU6050 mpu;
#define INTERRUPT_PIN 2

// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
float euler[3]; // [psi, theta, phi]    Euler angle container

// interruption logic
volatile bool mpuInterrupt = false;          // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; } // sets mpuInterrupt to true

// metros for motor movement
Metro leftMotorMetro;
Metro rightMotorMetro;

int k;

void MoveMotors()
{
  if (leftMotorMetro.check()){
    MotorLeft.makeStep();
    k++;
  }

  if (rightMotorMetro.check())
    MotorRight.makeStep();
}

unsigned long time;
void setup()
{
  setpoint=0;
  outputConstraint = value - maxSpeed;
  // initialize motors
  MotorLeft.connectToPins(50, 51);
  MotorRight.connectToPins(52, 53);
  MotorLeft.setDirection(Clockwise);
  MotorRight.setDirection(CounterClockwise);
  // set speed
  // MotorLeft.setSpeed(2000);
  // MotorRight.setSpeed(2000);
  // // set acceleration
  // MotorLeft.setAcceleration(20000);
  // MotorRight.setAcceleration(20000);

  // MotorLeft.setupStop();
  // MotorRight.setupStop();

  // begin HC-05 serial communication
  Serial1.begin(9600);

  Wire.begin();
  Wire.setClock(400000);

  // initialize serial communication
  Serial.begin(115200);

  // initalize mpu
  mpu.initialize();
  // set interruptpin mode
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  mpu.dmpInitialize();

  // gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(86);
  mpu.setYGyroOffset(21);
  mpu.setZGyroOffset(-17);
  mpu.setZAccelOffset(1032);

  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  // turn on the DMP, now that it's ready
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();

  // timer interruption
  Timer1.initialize(40);
  Timer1.attachInterrupt(MoveMotors);
}

void loop()
{
  bool serial = false;
  // if there is a package in stack continue with the flow of the program
  if (fifoCount < packetSize)
  {
    // if there has been an interruption read the package size in the stack
    if (mpuInterrupt)
      fifoCount = mpu.getFIFOCount();
    return;
  }

  // read HC-05 incoming data
  if (Serial1.available() > 0)
  {
    serial = true;
    Buffer = Serial1.read();
  }

  // reset interrupt flag
  mpuInterrupt = false;

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // read latest packet from FIFO
  while (fifoCount >= packetSize)
  {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;
  }

  // get current orientation quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  // display quaternion as Euler's angles
  mpu.dmpGetEuler(euler, &q);

  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // calculate PID
  input = ypr[1]* 180 / M_PI; // make rotation on yaw axis input for the PID controller

  error = setpoint - input; // calculate error

  // if (output > 10 || output < -10)
  //   error += output * 0.015; // brake function

  accumulatedError += Ki * error;                                            // calculate Integral part
  accumulatedError = constrain(accumulatedError, -iConstraint, iConstraint); // constrain Integral part

  output = (Kp * error) + accumulatedError + (Kd * (error - lastError)); // calculate output
  output = constrain(output, -400, 400);       // constrain output

  lastError = error; // set last error for next calculatuion

  //  output = abs(output) < 5 ? 0 : output; //add deadband
  if (output > 0)
    output = 405 - (1 / (output +9)) * 5500;
  else if (output < 0)
    output = -405 - (1 / (output-9)) * 5500;


  // set the directin of the motors
  MotorLeft.setDirection(output >= 0 ? Clockwise : CounterClockwise);
  MotorRight.setDirection(output < 0 ? Clockwise : CounterClockwise);
  
  

  float left=output;
  float right=output;

  if(Buffer == 'R'){
    left+=30;
    right-=30;
  }
  if(Buffer == 'L'){
    left-=30;
    right+=30;
  }

  if(Buffer == 'U'){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(setpoint > -5.5)setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(output > 150 * -1)setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
    Serial.println(setpoint);
  }
  if(Buffer == 'D'){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(setpoint < 5.5)setpoint += 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(output < 150)setpoint += 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
    Serial.println(setpoint);
  }

  if(Buffer == 'S'){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(setpoint > 0.5)setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(setpoint < -0.5)setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }

  left = abs(left)*81.73;
  left = constrain(left, -outputConstraint, outputConstraint);       // constrain output

  right = abs(right)*81.73;
  right = constrain(right, -outputConstraint, outputConstraint);       // constrain output

  leftMotorMetro.setInterval(value - left);
  rightMotorMetro.setInterval(value - right);
  k=0;
  // set the speed of the motors
  // leftMotorMetro.setInterval(value);
  // rightMotorMetro.setInterval(value);

}
