#include <Arduino.h>

#include "sample.h"

namespace sampling {

Sample::Sample(String timestamp, float airFlow, float airFlowCFM,
               float airPressure, float airTemperature, float airVelocity,
               float fanFrequency, bool fanState, float turbineAxialForce,
               float turbineTransverseForce, float turbinePower)
    : timestamp(timestamp), airFlow(airFlow), airFlowCFM(airFlowCFM),
      airPressure(airPressure), airTemperature(airTemperature),
      airVelocity(airVelocity), fanFrequency(fanFrequency), fanState(fanState),
      turbineAxialForce(turbineAxialForce),
      turbineTransverseForce(turbineTransverseForce),
      turbinePower(turbinePower) {}

Sample::~Sample() {}

String Sample::GetTimestamp() const { return this->timestamp; }

float Sample::GetAirFlow() const { return this->airFlow; }

float Sample::GetAirFlowCFM() const { return this->airFlowCFM; }

float Sample::GetAirPressure() const { return this->airPressure; }

float Sample::GetAirTemperature() const { return this->airTemperature; }

float Sample::GetAirVelocity() const { return this->airVelocity; }

float Sample::GetFanFrequency() const { return this->fanFrequency; }

bool Sample::GetFanState() const { return this->fanState; }

float Sample::GetTurbineAxialForce() const { return this->turbineAxialForce; }

float Sample::GetTurbineTransverseForce() const {
  return this->turbineTransverseForce;
}

float Sample::GetTurbinePower() const { return this->turbinePower; }

} // namespace sampling