#include "measurements.h"

namespace control {

Measures::Measures(float airTemperature, float airRelativeHumidity,
                   float airVelocity, float dynamicPressure, float airFlow,
                   float airFlowCfm, float airDensity, float windTurbineRpm,
                   float windTurbinePower, float fanFrequencySetPoint,
                   float fanRpm, float fanPower)
    : airTemperature(airTemperature), airRelativeHumidity(airRelativeHumidity),
      airVelocity(airVelocity), dynamicPressure(dynamicPressure),
      airFlow(airFlow), airFlowCfm(airFlowCfm), airDensity(airDensity),
      windTurbineRpm(windTurbineRpm), windTurbinePower(windTurbinePower),
      fanFrequencySetPoint(fanFrequencySetPoint), fanRpm(fanRpm),
      fanPower(fanPower) {}

Measures::~Measures() {}

float Measures::GetAirTemperature() { return this->airTemperature; }

float Measures::GetAirRelativeHumidity() { return this->airRelativeHumidity; }

float Measures::GetAirVelocity() { return this->airVelocity; }

float Measures::GetDynamicPressure() { return this->dynamicPressure; }

float Measures::GetAirFlow() { return this->airFlow; }

float Measures::GetAirFlowCfm() { return this->airFlowCfm; }

float Measures::GetAirDensity() { return this->airDensity; }

float Measures::GetWindTurbineRpm() { return this->windTurbineRpm; }

float Measures::GetWindTurbinePower() { return this->windTurbinePower; }

float Measures::GetFanFrequencySetPoint() { return this->fanFrequencySetPoint; }

float Measures::GetFanRpm() { return this->fanRpm; }

float Measures::GetFanPower() { return this->fanPower; }

} // namespace control