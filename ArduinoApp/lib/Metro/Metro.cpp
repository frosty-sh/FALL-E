
#include "Arduino.h"
#include "Metro.h"

// #define DEBUG

Metro::Metro()
{
  this->interval = 0;
  this->previous = micros();
}

void Metro::setInterval(unsigned long interval)
{
  this->interval = interval;
}

bool Metro::check()
{
  unsigned long now = micros();

  if ((interval + previous) > now || interval == 0)
    return false;

  previous = now;
  return true;
}