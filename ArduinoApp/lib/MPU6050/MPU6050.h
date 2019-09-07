#ifndef MPU6050_H_
#define MPU6050_H_
#include<Arduino.h>

class MPU6050
{

    int _Adress;

public:
    MPU6050(int Adress);
    int16_t GetXAngle();
};

#endif