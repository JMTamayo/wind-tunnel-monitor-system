#include "controller.h"

namespace control {

Measures::Measures(peripherals::AirProperties airProperties,
                   peripherals::RotationalFrequency windTurbineProperties,
                   peripherals::FanProperties fanProperties)
    : airProperties(airProperties),
      windTurbineProperties(windTurbineProperties),
      fanProperties(fanProperties) {}

Measures::~Measures() {}

peripherals::AirProperties Measures::GetAirProperties() {
  return this->airProperties;
}

peripherals::RotationalFrequency Measures::GetWindTurbineProperties() {
  return this->windTurbineProperties;
}

peripherals::FanProperties Measures::GetFanProperties() {
  return this->fanProperties;
}

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

peripherals::BermE3fDs30c4 *Controller::getBermE3fDs30c4() {
  return this->bermE3fDs30c4;
}

peripherals::Dht22 *Controller::getDht22() { return this->dht22; }

peripherals::Fan *Controller::getFan() { return this->fan; }

Controller::Controller(const unsigned long intervalMs,
                       const unsigned int windTurbinePulseCountDivider,
                       peripherals::BermE3fDs30c4 *bermE3fDs30c4,
                       peripherals::Dht22 *dht22, peripherals::Fan *fan)
    : intervalMs(intervalMs), lastMeasurementTimeMs(millis()),
      windTurbinePulseCountDivider(windTurbinePulseCountDivider),
      bermE3fDs30c4(bermE3fDs30c4), dht22(dht22), fan(fan) {}

Controller::~Controller() {}

void Controller::Begin() { this->getBermE3fDs30c4()->Restart(); }

bool Controller::IsMeasurementTimeReached() {
  return millis() - this->getLastMeasurementTimeMs() >= this->getIntervalMs();
}

Measures Controller::Measure() {
  peripherals::AirProperties airProperties = this->getDht22()->Read();
  peripherals::FanProperties fanProperties = this->getFan()->Read();
  peripherals::RotationalFrequency windTurbineProperties =
      this->getBermE3fDs30c4()->Read(this->getWindTurbinePulseCountDivider());

  this->setLastMeasurementTimeMs(millis());

  return Measures(airProperties, windTurbineProperties, fanProperties);
}

void Controller::SetFanFrequency(float frequency) {
  this->getFan()->SetFrequency(frequency);
}

} // namespace control
