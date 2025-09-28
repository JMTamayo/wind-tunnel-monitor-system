#ifndef BERM_E3F_DS30C4_H
#define BERM_E3F_DS30C4_H

#include <Arduino.h>

namespace peripherals {

class RotationalFrequency {
private:
  unsigned long pulseCount;
  unsigned long sampleTimeSeconds;
  float rotationalFrequencyHz;
  float rotationalFrequencyRpm;

public:
  RotationalFrequency(unsigned long pulseCount, unsigned long sampleTimeSeconds,
                      float rotationalFrequencyHz,
                      float rotationalFrequencyRpm);

  ~RotationalFrequency();

  unsigned long GetPulseCount();

  unsigned long GetSampleTimeSeconds();

  float GetRotationalFrequencyHz();

  float GetRotationalFrequencyRpm();
};

extern volatile unsigned long pulseCounter;

void IRAM_ATTR incrementPulseCount();

class BermE3fDs30c4 {
private:
  const unsigned int pin;

  unsigned long lastMeasurementTimeMs;

  const unsigned int getPin() const;

  unsigned long getLastMeasurementTimeMs() const;

  void setLastMeasurementTimeMs(unsigned long lastMeasurementTimeMs);

public:
  BermE3fDs30c4(const unsigned int pin);

  ~BermE3fDs30c4();

  void Restart();

  RotationalFrequency Read(unsigned int pulseCountDivider);
};

} // namespace peripherals

#endif // BERM_E3F_DS30C4_H