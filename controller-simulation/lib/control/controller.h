#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

#include "logger.h"

#include "measurements.h"

#include "berm_e3f_ds30c4.h"
#include "dht22.h"
#include "mpxv7002dp_adc1115.h"

namespace control {

class Controller {
private:
  const unsigned long intervalMs;
  unsigned long lastMeasurementTimeMs;

  const unsigned int windTurbinePulseCountDivider;
  const float crossSectionArea;
  const float airDensity;
  const float cubicMetersPerSecondToCubicFeetPerMinute;

  float lastFanFrequencySetPoint;

  peripherals::BermE3fDs30c4 *bermE3fDs30c4;
  peripherals::Dht22 *dht22;
  peripherals::MPXV7002DP_ADC1115 *mpxv7002dp_Adc1115;

  const unsigned long getIntervalMs() const;

  unsigned long getLastMeasurementTimeMs() const;

  void setLastMeasurementTimeMs(const unsigned long lastMeasurementTimeMs);

  const unsigned int getWindTurbinePulseCountDivider() const;

  const float getCrossSectionArea() const;

  const float getAirDensity() const;

  const float getCubicMetersPerSecondToCubicFeetPerMinute() const;

  float getLastFanFrequencySetPoint() const;

  void setLastFanFrequencySetPoint(float lastFanFrequencySetPoint);

  peripherals::BermE3fDs30c4 *getBermE3fDs30c4();

  peripherals::Dht22 *getDht22();

  peripherals::MPXV7002DP_ADC1115 *getMpxv7002dp_Adc1115();

  float getWindTurbinePower(float rotationalFrequencyRpm) const;

public:
  Controller(unsigned long intervalMs,
             unsigned int windTurbinePulseCountDivider, float airDensity,
             float crossSectionArea,
             float cubicMetersPerSecondToCubicFeetPerMinute,
             peripherals::BermE3fDs30c4 *bermE3fDs30c4,
             peripherals::Dht22 *dht22,
             peripherals::MPXV7002DP_ADC1115 *mpxv7002dp_Adc1115);

  ~Controller();

  void Begin();

  bool IsMeasurementTimeReached();

  Measures Measure();

  void SetFanFrequency(float frequency);
};

} // namespace control

#endif // CONTROLLER_H
