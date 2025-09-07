#ifndef SAMPLE_H
#define SAMPLE_H

#include <Arduino.h>

namespace sampling {

class Sample {
private:
  String timestamp;
  float airFlow;
  float airFlowCFM;
  float airPressure;
  float airTemperature;
  float airVelocity;
  float fanFrequency;
  bool fanState;
  float turbineAxialForce;
  float turbineTransverseForce;
  float turbinePower;

public:
  Sample(String timestamp, float airFlow, float airFlowCFM, float airPressure,
         float airTemperature, float airVelocity, float fanFrequency,
         bool fanState, float turbineAxialForce, float turbineTransverseForce,
         float turbinePower);

  ~Sample();

  String GetTimestamp() const;

  float GetAirFlow() const;

  float GetAirFlowCFM() const;

  float GetAirPressure() const;

  float GetAirTemperature() const;

  float GetAirVelocity() const;

  float GetFanFrequency() const;

  bool GetFanState() const;

  float GetTurbineAxialForce() const;

  float GetTurbineTransverseForce() const;

  float GetTurbinePower() const;
};

} // namespace sampling

#endif // SAMPLE_H