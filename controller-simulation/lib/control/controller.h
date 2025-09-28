#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

#include "berm_e3f_ds30c4.h"
#include "dht22.h"
#include "fan.h"

namespace control {

class Measures {
private:
  peripherals::AirProperties airProperties;
  peripherals::RotationalFrequency windTurbineProperties;
  peripherals::FanProperties fanProperties;

public:
  Measures(peripherals::AirProperties airProperties,
           peripherals::RotationalFrequency windTurbineProperties,
           peripherals::FanProperties fanProperties);

  ~Measures();

  peripherals::AirProperties GetAirProperties();

  peripherals::RotationalFrequency GetWindTurbineProperties();

  peripherals::FanProperties GetFanProperties();
};

class Controller {
private:
  const unsigned long intervalMs;
  unsigned long lastMeasurementTimeMs;

  const unsigned int windTurbinePulseCountDivider;

  peripherals::BermE3fDs30c4 *bermE3fDs30c4;
  peripherals::Dht22 *dht22;
  peripherals::Fan *fan;

  const unsigned long getIntervalMs() const;

  unsigned long getLastMeasurementTimeMs() const;

  void setLastMeasurementTimeMs(const unsigned long lastMeasurementTimeMs);

  const unsigned int getWindTurbinePulseCountDivider() const;

  peripherals::BermE3fDs30c4 *getBermE3fDs30c4();

  peripherals::Dht22 *getDht22();

  peripherals::Fan *getFan();

public:
  Controller(unsigned long intervalMs,
             unsigned int windTurbinePulseCountDivider,
             peripherals::BermE3fDs30c4 *bermE3fDs30c4,
             peripherals::Dht22 *dht22, peripherals::Fan *fan);

  ~Controller();

  void Begin();

  bool IsMeasurementTimeReached();

  Measures Measure();

  void SetFanFrequency(float frequency);
};

} // namespace control

#endif // CONTROLLER_H
