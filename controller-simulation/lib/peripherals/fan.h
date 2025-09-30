#ifndef FAN_H
#define FAN_H

#include <Arduino.h>

#include "logger.h"

namespace peripherals {

class FanProperties {
private:
  float rpm;
  float frequencySetPoint;

public:
  FanProperties(float rpm, float frequencySetPoint);

  ~FanProperties();

  float GetRpm();

  float GetFrequencySetPoint();
};

class Fan {
private:
  const unsigned int controlPin;
  float frequencySetPoint;

  const unsigned int getControlPin() const;

  void setFrequencySetPoint(float frequencySetPoint);

public:
  Fan(const unsigned int controlPin);

  ~Fan();

  float GetFrequencySetPoint() const;

  void SetFrequency(float frequency);

  FanProperties Read();
};

} // namespace peripherals

#endif // FAN_H