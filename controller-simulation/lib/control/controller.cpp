#include "controller.h"

namespace control {

Measures::Measures(float airTemperature, float airRelativeHumidity,
                   float airVelocity, float dynamicPressure, float airFlow,
                   float airFlowCfm, float airDensity, float windTurbineRpm,
                   float windTurbinePower, float fanFrequencySetPoint,
                   float fanRpm)
    : airTemperature(airTemperature), airRelativeHumidity(airRelativeHumidity),
      airVelocity(airVelocity), dynamicPressure(dynamicPressure),
      airFlow(airFlow), airFlowCfm(airFlowCfm), airDensity(airDensity),
      windTurbineRpm(windTurbineRpm), windTurbinePower(windTurbinePower),
      fanFrequencySetPoint(fanFrequencySetPoint), fanRpm(fanRpm) {}

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

const unsigned long Controller::getIntervalMs() const {
  return this->intervalMs;
}

unsigned long Controller::getLastMeasurementTimeMs() const {
  return this->lastMeasurementTimeMs;
}

void Controller::setLastMeasurementTimeMs(
    const unsigned long lastMeasurementTimeMs) {
  this->lastMeasurementTimeMs = lastMeasurementTimeMs;
}

const unsigned int Controller::getWindTurbinePulseCountDivider() const {
  return this->windTurbinePulseCountDivider;
}

const float Controller::getCrossSectionArea() const {
  return this->crossSectionArea;
}

const float Controller::getAirDensity() const { return this->airDensity; }

peripherals::BermE3fDs30c4 *Controller::getBermE3fDs30c4() {
  return this->bermE3fDs30c4;
}

peripherals::Dht22 *Controller::getDht22() { return this->dht22; }

peripherals::MPXV7002DP_ADC1115 *Controller::getMpxv7002dp_Adc1115() {
  return this->mpxv7002dp_Adc1115;
}

peripherals::Fan *Controller::getFan() { return this->fan; }

Controller::Controller(const unsigned long intervalMs,
                       const unsigned int windTurbinePulseCountDivider,
                       const float airDensity, const float crossSectionArea,
                       peripherals::BermE3fDs30c4 *bermE3fDs30c4,
                       peripherals::Dht22 *dht22,
                       peripherals::MPXV7002DP_ADC1115 *mpxv7002dp_Adc1115,
                       peripherals::Fan *fan)
    : intervalMs(intervalMs), lastMeasurementTimeMs(millis()),
      windTurbinePulseCountDivider(windTurbinePulseCountDivider),
      airDensity(airDensity), crossSectionArea(crossSectionArea),
      bermE3fDs30c4(bermE3fDs30c4), dht22(dht22),
      mpxv7002dp_Adc1115(mpxv7002dp_Adc1115), fan(fan) {}

float Controller::getWindTurbinePower(float rotationalFrequencyRpm) const {
  // The following values are for testing purposes. You should delete this when
  // the real prototype is ready.
  return 0.0034f * rotationalFrequencyRpm;
}

Controller::~Controller() {}

void Controller::Begin() {
  this->getBermE3fDs30c4()->Begin();
  this->getDht22()->Begin();
  this->getMpxv7002dp_Adc1115()->Begin();
}

bool Controller::IsMeasurementTimeReached() {
  return millis() - this->getLastMeasurementTimeMs() >= this->getIntervalMs();
}

Measures Controller::Measure() {
  peripherals::AirProperties airProperties = this->getDht22()->Read();
  peripherals::FanProperties fanProperties = this->getFan()->Read();
  peripherals::RotationalFrequency windTurbineProperties =
      this->getBermE3fDs30c4()->Read(this->getWindTurbinePulseCountDivider());

  float windTurbinePower = this->getWindTurbinePower(
      windTurbineProperties.GetRotationalFrequencyRpm());

  float airDensity = this->getAirDensity();
  peripherals::DynamicPressure dynamicPressure =
      this->getMpxv7002dp_Adc1115()->Read(airDensity);

  float airVelocity = dynamicPressure.GetAirVelocity();
  float airFlow = airVelocity * this->getCrossSectionArea();
  float airFlowCfm = airFlow * 2118.8799727597;

  Measures measures(
      airProperties.GetTemperature(), airProperties.GetRelativeHumidity(),
      airVelocity, dynamicPressure.GetPressure(), airFlow, airFlowCfm,
      airDensity, windTurbineProperties.GetRotationalFrequencyRpm(),
      windTurbinePower, fanProperties.GetFrequencySetPoint(),
      fanProperties.GetRpm());

  this->setLastMeasurementTimeMs(millis());
  return measures;
}

void Controller::SetFanFrequency(float frequency) {
  this->getFan()->SetFrequency(frequency);
}

} // namespace control
