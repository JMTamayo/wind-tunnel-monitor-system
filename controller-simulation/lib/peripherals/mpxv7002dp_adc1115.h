#ifndef MPXV7002DP_ADC1115_H
#define MPXV7002DP_ADC1115_H

#include <Arduino.h>

namespace peripherals {

class DynamicPressure {
private:
  float pressure;
  float airVelocity;

public:
  DynamicPressure(float pressure, float airVelocity);

  ~DynamicPressure();

  float GetPressure();

  float GetAirVelocity();
};

class MPXV7002DP_ADC1115 {
private:
  unsigned int channel;
  float voltageOffset;
  float voltageSensitivity;

  unsigned int getChannel();

  float getVoltageOffset();

  float getVoltageSensitivity();

public:
  MPXV7002DP_ADC1115(unsigned int channel, float voltageOffset,
                     float voltageSensitivity);

  ~MPXV7002DP_ADC1115();

  DynamicPressure Read(float airDensity);
};

} // namespace peripherals

#endif // MPXV7002DP_ADC1115_H