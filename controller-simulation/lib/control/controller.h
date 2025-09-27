#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

#include "dht22.h"
#include "fan.h"

namespace control {

class Measures {
private:
  peripherals::AirProperties airProperties;

  peripherals::FanProperties fanProperties;

public:
  Measures(peripherals::AirProperties airProperties,
           peripherals::FanProperties fanProperties);

  ~Measures();

  peripherals::AirProperties GetAirProperties();

  peripherals::FanProperties GetFanProperties();
};

class Controller {
private:
  const unsigned long intervalMs;
  unsigned long lastMeasurementTimeMs;

  peripherals::Dht22 *dht22;

  peripherals::Fan *fan;

  const unsigned long getIntervalMs() const;

  unsigned long getLastMeasurementTimeMs() const;

  void setLastMeasurementTimeMs(const unsigned long lastMeasurementTimeMs);

  peripherals::Dht22 *getDht22();

  peripherals::Fan *getFan();

public:
  Controller(unsigned long intervalMs, peripherals::Dht22 *dht22,
             peripherals::Fan *fan);

  ~Controller();

  bool IsMeasurementTimeReached();

  Measures Measure();

  void SetFanFrequency(float frequency);
};

} // namespace control

#endif // CONTROLLER_H
