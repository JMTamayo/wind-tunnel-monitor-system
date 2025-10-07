#include "controller.h"

namespace control {

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

const float Controller::getCubicMetersPerSecondToCubicFeetPerMinute() const {
  return this->cubicMetersPerSecondToCubicFeetPerMinute;
}

float Controller::getLastFanFrequencySetPoint() const {
  return this->lastFanFrequencySetPoint;
}

void Controller::setLastFanFrequencySetPoint(float lastFanFrequencySetPoint) {
  this->lastFanFrequencySetPoint = lastFanFrequencySetPoint;
}

peripherals::BermE3fDs30c4 *Controller::getBermE3fDs30c4() {
  return this->bermE3fDs30c4;
}

peripherals::Dht22 *Controller::getDht22() { return this->dht22; }

peripherals::MPXV7002DP_ADC1115 *Controller::getMpxv7002dp_Adc1115() {
  return this->mpxv7002dp_Adc1115;
}

Controller::Controller(const unsigned long intervalMs,
                       const unsigned int windTurbinePulseCountDivider,
                       const float airDensity, const float crossSectionArea,
                       const float cubicMetersPerSecondToCubicFeetPerMinute,
                       peripherals::BermE3fDs30c4 *bermE3fDs30c4,
                       peripherals::Dht22 *dht22,
                       peripherals::MPXV7002DP_ADC1115 *mpxv7002dp_Adc1115)
    : intervalMs(intervalMs), lastMeasurementTimeMs(millis()),
      windTurbinePulseCountDivider(windTurbinePulseCountDivider),
      airDensity(airDensity), crossSectionArea(crossSectionArea),
      cubicMetersPerSecondToCubicFeetPerMinute(
          cubicMetersPerSecondToCubicFeetPerMinute),
      bermE3fDs30c4(bermE3fDs30c4), dht22(dht22),
      mpxv7002dp_Adc1115(mpxv7002dp_Adc1115) {
  /*
  TODO: Implement the real program to setup the controller.

  The following values are for testing purposes.
  */
  pinMode(23, OUTPUT);
  analogWrite(23, this->getLastFanFrequencySetPoint());
  /* --- */
}

float Controller::getWindTurbinePower(float rotationalFrequencyRpm) const {
  /*
  TODO: Implement the real program to get the wind turbine power.

  The value of the generated power is a function of the rotation frequency in
  RPM

  The following values are for testing purposes. You should delete this when
  the real prototype is ready.
  */
  return 0.0034f * rotationalFrequencyRpm;
  /* --- */
}

Controller::~Controller() {}

void Controller::Begin() {
  this->getBermE3fDs30c4()->Begin();
  this->getDht22()->Begin();
}

bool Controller::IsMeasurementTimeReached() {
  return millis() - this->getLastMeasurementTimeMs() >= this->getIntervalMs();
}

Measures Controller::Measure() {
  peripherals::AirProperties airProperties = this->getDht22()->Read();
  peripherals::RotationalFrequency windTurbineProperties =
      this->getBermE3fDs30c4()->Read(this->getWindTurbinePulseCountDivider());

  float windTurbinePower = this->getWindTurbinePower(
      windTurbineProperties.GetRotationalFrequencyRpm());

  float airDensity = this->getAirDensity();
  peripherals::DynamicPressure dynamicPressure =
      this->getMpxv7002dp_Adc1115()->Read(airDensity);

  float airVelocity = dynamicPressure.GetAirVelocity();
  float airFlow = airVelocity * this->getCrossSectionArea();
  float airFlowCfm =
      airFlow * this->getCubicMetersPerSecondToCubicFeetPerMinute();

  /*
  TODO: Implement the real program to read the fan properties: frequency set
  point, rpm, and total power.

  The following values are for testing purposes.
  */
  float fanFrequencySetPoint =
      max(0.0f, this->getLastFanFrequencySetPoint() + random(-2, 0));
  float fanRpm = random(1000, 1050);
  float fanPower = random(300, 340) / 100.0;
  /* --- */

  Measures measures(
      airProperties.GetTemperature(), airProperties.GetRelativeHumidity(),
      airVelocity, dynamicPressure.GetPressure(), airFlow, airFlowCfm,
      airDensity, windTurbineProperties.GetRotationalFrequencyRpm(),
      windTurbinePower, fanFrequencySetPoint, fanRpm, fanPower);

  this->setLastMeasurementTimeMs(millis());
  return measures;
}

void Controller::SetFanFrequency(float frequency) {
  /*
  TODO: Implement the real program to set the fan frequency.
  */
  float constrainedFrequency = constrain(frequency, 0.0f, 100.0f);
  unsigned int mappedFrequency =
      map(constrainedFrequency, 0.0f, 100.0f, 0, 255);

  logging::logger->Debug("Setting fan frequency. Requested frequency: " +
                         String(constrainedFrequency) + "%.");

  this->setLastFanFrequencySetPoint(constrainedFrequency);
  analogWrite(23, mappedFrequency);
  /* --- */
}

} // namespace control
