#ifndef FAN_H
#define FAN_H

#include <Arduino.h>

#include "logger.h"

namespace peripherals {

class FanProperties {
private:
  float rpm;

public:
  FanProperties(float rpm);

  ~FanProperties();

  float GetRpm();
};

class Fan {
private:
  const unsigned int controlPin;
  float setPointFrequency;

  const unsigned int getControlPin() const;

  float getSetPointFrequency() const;

  void setSetPointFrequency(float setPointFrequency);

public:
  Fan(const unsigned int controlPin);

  ~Fan();

  void SetFrequency(float frequency);

  FanProperties Read();
};

} // namespace peripherals

#endif // FAN_H