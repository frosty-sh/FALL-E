#include <MPU6050.h>
#include <Wire.h>

MPU6050::MPU6050(int Adress)
{
  _Adress = Adress;

  Wire.begin();
  Wire.beginTransmission(_Adress); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B);                // PWR_MGMT_1 register
  Wire.write(0);                   // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

int16_t MPU6050::GetXAngle()
{
  Wire.beginTransmission(_Adress);
  Wire.write(0x3B);                       // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false);            // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(_Adress, 7 * 2, true); // request a total of 7*2=14 registers

  return Wire.read() << 8 | Wire.read();
}