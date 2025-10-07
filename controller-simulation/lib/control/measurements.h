#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

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
  float fanPower;

public:
  Measures(float airTemperature, float airRelativeHumidity, float airVelocity,
           float dynamicPressure, float airFlow, float airFlowCfm,
           float airDensity, float windTurbineRpm, float windTurbinePower,
           float fanFrequencySetPoint, float fanRpm, float fanPower);

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

  float GetFanPower();
};

} // namespace control

#endif // MEASUREMENTS_H