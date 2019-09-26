#ifndef PID__H
#define PID__H

class PID
{
  public:
  float input, setpoint, error, output;
  float Kp;
  float Ki, accumulatedError;
  float Kd, lastError;
};

#endif

