#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

#include "dht22.h"

namespace control {

class Measures {
private:
  peripherals::AirProperties airProperties;

public:
  Measures(peripherals::AirProperties airProperties);

  ~Measures();

  peripherals::AirProperties GetAirProperties();
};

class Controller {
private:
  const unsigned long intervalMs;
  unsigned long lastMeasurementTimeMs;

  peripherals::Dht22 *dht22;

  const unsigned long getIntervalMs() const;

  unsigned long getLastMeasurementTimeMs() const;

  void setLastMeasurementTimeMs(const unsigned long lastMeasurementTimeMs);

  peripherals::Dht22 *getDht22();

public:
  Controller(unsigned long intervalMs, peripherals::Dht22 *dht22);

  ~Controller();

  bool IsMeasurementTimeReached();

  Measures Measure();
};

} // namespace control

#endif // CONTROLLER_H
