#ifndef FAN_H
#define FAN_H

#include <Arduino.h>

#include "logger.h"

#include "sdm630mct.h"

namespace peripherals {

class FanProperties {
private:
  float rpm;
  float frequencySetPoint;
  float totalActivePower;

public:
  FanProperties(float rpm, float frequencySetPoint, float totalActivePower);

  ~FanProperties();

  float GetRpm();

  float GetFrequencySetPoint();

  float GetTotalActivePower();
};

class Fan {
private:
  const unsigned int controlPin;
  float frequencySetPoint;

  peripherals::SDM630MCT *sdm630mct;

  const unsigned int getControlPin() const;

  void setFrequencySetPoint(float frequencySetPoint);

  peripherals::SDM630MCT *getSdm630mct();

public:
  Fan(const unsigned int controlPin, peripherals::SDM630MCT *sdm630mct);

  ~Fan();

  void Begin();

  float GetFrequencySetPoint() const;

  void SetFrequency(float frequency);

  FanProperties Read();
};

} // namespace peripherals

#endif // FAN_H