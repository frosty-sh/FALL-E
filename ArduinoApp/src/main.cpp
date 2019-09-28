// #include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Motor.h>
#include <TimerOne.h>
#include <Metro.h>

//PID vars
float input, setpoint, error, output, outputConstraint, deadband = 5;
float Kp = 28;                                     // proportional part
float Ki = 3, accumulatedError, iConstraint = 400; // integral part
float Kd = 0.8, lastError;                         // derivative part

MPU6050 mpu;
#define INTERRUPT_PIN 2

// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
VectorFloat gravity;    // gravity vector
float ypr[3];           // yaw, pitch and roll array

// orientation/motion vars
Quaternion q;           // quaternion container
VectorFloat gravity;    // gravity vector
float ypr[3];           // yaw, pitch and roll array

// interruption logic
volatile bool mpuInterrupt = false;          // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { mpuInterrupt = true; } // sets mpuInterrupt to true

// metros for motor movement
Metro leftMotorMetro;
Metro rightMotorMetro;

// motors
Motor motorLeft;
Motor motorRight;
int maxSpeed = 300;   // minimum delay between steps
int minSpeed = 32000; // maximum delay between steps

// movement var
char serialData = 'S'; // current movement command

// interruption function to make motor steps when the time is right
void MoveMotors()
{
  if (leftMotorMetro.check())
    motorLeft.makeStep();

  if (rightMotorMetro.check())
    motorRight.makeStep();
}

void setup()
{
  setpoint = 0;                           // set pid setpoint
  outputConstraint = minSpeed - maxSpeed; //set pid output constraint
  // initialize motors
  motorLeft.connectToPins(50, 51);
  motorRight.connectToPins(52, 53);
  motorLeft.setDirection(Clockwise);
  motorRight.setDirection(CounterClockwise);

  Serial1.begin(9600); // begin HC-05 serial communication

  // setup communicaton with MPU6050
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200); // initialize serial communication

  mpu.initialize();              // initalize mpu
  pinMode(INTERRUPT_PIN, INPUT); // set interruptpin mode

  mpu.dmpInitialize(); // load and configure the DMP

  // gyro offsets, scaled for min sensitivity
  mpu.setXGyroOffset(86);
  mpu.setYGyroOffset(21);
  mpu.setZGyroOffset(-17);
  mpu.setZAccelOffset(1032);

  // generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); // enable Arduino interrupt detection

  packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison

  // timer interruption
  Timer1.initialize(40);
  Timer1.attachInterrupt(MoveMotors);
}

void loop()
{ 
  ////////////////////////////////////////////////////////////////////////////////////
  /// Read MPU6050 data                                                            ///
  ////////////////////////////////////////////////////////////////////////////////////

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
    serialData = Serial1.read();
  }

  mpuInterrupt = false; // reset interrupt flag

  fifoCount = mpu.getFIFOCount(); // get current FIFO count

  // read latest packet from FIFO
  while (fifoCount >= packetSize)
  {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer); // get current orientation quaternion

  // get yaw pitch and roll
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  ////////////////////////////////////////////////////////////////////////////////////
  /// Calculate PID                                                                ///
  ////////////////////////////////////////////////////////////////////////////////////

  input = ypr[1] * 180 / M_PI; // make rotation on yaw axis input for the PID controller
  error = setpoint - input;    // calculate error

  // if (output > 10 || output < -10)
  //   error += output * 0.015; // brake function

  accumulatedError += Ki * error;                                            // calculate Integral part
  accumulatedError = constrain(accumulatedError, -iConstraint, iConstraint); // constrain Integral part
  
  output = (Kp * error) + accumulatedError + (Kd * (error - lastError));     // calculate output
  output = constrain(output, -400, 400);                                     // constrain output
 
  lastError = error;                                                         // set last error for next calculatuion
 
  output = abs(output) < 5 ? 0 : output;                                     // add deadband
 
  // conmpensate for the non-linear behaviour of stepper motor speed
  if (output > 0)
    output = 405 - (1 / (output + 9)) * 5500;
  else if (output < 0)
    output = -405 - (1 / (output - 9)) * 5500;

  ////////////////////////////////////////////////////////////////////////////////////
  /// Generate motor speeds                                                        ///
  ////////////////////////////////////////////////////////////////////////////////////

  // initialize left and right output
  float left = output;
  float right = output;

  // steer right
  if (serialData == 'R')
  {
    left += 30;  // increase left motor speed
    right -= 30; // decrease right motor speed
  }
  // steer left
  if (serialData == 'L')
  {
    left -= 30;  // decrease left motor speed
    right += 30; // increase right motor speed
  }

  // move forward
  if (serialData == 'U')
  {
    if (setpoint > -5.5)
      setpoint -= 0.05;     // slowly change the setpoint angle so the robot starts leaning forewards
    if (output > 150 * -1)  //
      setpoint -= 0.005;    // slowly change the setpoint angle so the robot starts leaning forewards
  }

  // move backwards
  if (serialData == 'D')
  {
    if (setpoint < 5.5)
      setpoint += 0.05;     // slowly change the setpoint angle so the robot starts leaning forewards
    if (output < 150)       //
      setpoint += 0.005;    // slowly change the setpoint angle so the robot starts leaning forewards
  }

  // stop movement
  if (serialData == 'S')
  {
    if (setpoint > 0.5)
      setpoint -= 0.05;       // if the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if (setpoint < -0.5) //
      setpoint += 0.05;       //if the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else                      //
      setpoint = 0;           //if the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }

  ////////////////////////////////////////////////////////////////////////////////////
  /// Set motor speed and direction                                                ///
  ////////////////////////////////////////////////////////////////////////////////////

  // set the directin of the motors
  motorLeft.setDirection(output >= 0 ? Clockwise : CounterClockwise);
  motorRight.setDirection(output < 0 ? Clockwise : CounterClockwise);

  left = abs(left) * 81.73;                                      // scale output for 1/16 stepp
  left = constrain(left, -outputConstraint, outputConstraint);   // constrain output

  right = abs(right) * 81.73;                                    // scale output for 1/16 stepp
  right = constrain(right, -outputConstraint, outputConstraint); // constrain output

  leftMotorMetro.setInterval(minSpeed - left);                   // set the metro interval for the left motor
  rightMotorMetro.setInterval(minSpeed - right);                 // set the metro interval for the right motor
}
