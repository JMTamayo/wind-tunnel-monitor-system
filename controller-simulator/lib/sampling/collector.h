#ifndef COLLECTOR_H
#define COLLECTOR_H

#include "time_.h"

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

class Collector {
private:
  unsigned long intervalMs;
  unsigned long lastSampleTimeMs;

  services::TimeService *timeService;

  unsigned long getIntervalMs() const;

  unsigned long getLastSampleTimeMs() const;

  void setLastSampleTimeMs(unsigned long lastSampleTimeMs);

  services::TimeService *getTimeService() const;

public:
  Collector(unsigned long intervalMs, services::TimeService *timeService);

  ~Collector();

  bool IsSamplingTimeReached();

  Sample Collect();
};

} // namespace sampling

#endif // COLLECTOR_H