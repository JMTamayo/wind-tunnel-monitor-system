#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

#include "berm_e3f_ds30c4.h"
#include "dht22.h"
#include "fan.h"
#include "mpxv7002dp_adc1115.h"

namespace control {

class Measures {
private:
  float airTemperature;
  float airRelativeHumidity;

  float airVelocity;
  float airDensity;
  float dynamicPressure;

  float airFlow;
  float airFlowCfm;

  float windTurbineRpm;
  float windTurbinePower;

  float fanFrequencySetPoint;
  float fanRpm;

public:
  Measures(float airTemperature, float airRelativeHumidity, float airVelocity,
           float dynamicPressure, float airFlow, float airFlowCfm,
           float airDensity, float windTurbineRpm, float windTurbinePower,
           float fanFrequencySetPoint, float fanRpm);

  ~Measures();

  float GetAirTemperature();

  float GetAirRelativeHumidity();

  float GetAirVelocity();

  float GetAirDensity();

  float GetDynamicPressure();

  float GetAirFlow();

  float GetAirFlowCfm();

  float GetWindTurbineRpm();

  float GetWindTurbinePower();

  float GetFanFrequencySetPoint();

  float GetFanRpm();
};

class Controller {
private:
  const unsigned long intervalMs;
  unsigned long lastMeasurementTimeMs;

  const unsigned int windTurbinePulseCountDivider;

  const float crossSectionArea;
  const float airDensity;

  peripherals::BermE3fDs30c4 *bermE3fDs30c4;
  peripherals::Dht22 *dht22;
  peripherals::MPXV7002DP_ADC1115 *mpxv7002dp_Adc1115;

  peripherals::Fan *fan;

  const unsigned long getIntervalMs() const;

  unsigned long getLastMeasurementTimeMs() const;

  void setLastMeasurementTimeMs(const unsigned long lastMeasurementTimeMs);

  const unsigned int getWindTurbinePulseCountDivider() const;

  const float getCrossSectionArea() const;

  const float getAirDensity() const;

  peripherals::BermE3fDs30c4 *getBermE3fDs30c4();

  peripherals::Dht22 *getDht22();

  peripherals::MPXV7002DP_ADC1115 *getMpxv7002dp_Adc1115();

  peripherals::Fan *getFan();

  float getWindTurbinePower(float rotationalFrequencyRpm) const;

public:
  Controller(unsigned long intervalMs,
             unsigned int windTurbinePulseCountDivider, float airDensity,
             float crossSectionArea, peripherals::BermE3fDs30c4 *bermE3fDs30c4,
             peripherals::Dht22 *dht22,
             peripherals::MPXV7002DP_ADC1115 *mpxv7002dp_Adc1115,
             peripherals::Fan *fan);

  ~Controller();

  void Begin();

  bool IsMeasurementTimeReached();

  Measures Measure();

  void SetFanFrequency(float frequency);
};

} // namespace control

#endif // CONTROLLER_H
