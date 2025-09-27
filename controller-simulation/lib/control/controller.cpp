#include "controller.h"

namespace control {

Measures::Measures(peripherals::AirProperties airProperties,
                   peripherals::FanProperties fanProperties)
    : airProperties(airProperties), fanProperties(fanProperties) {}

Measures::~Measures() {}

peripherals::AirProperties Measures::GetAirProperties() {
  return this->airProperties;
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

peripherals::Dht22 *Controller::getDht22() { return this->dht22; }

peripherals::Fan *Controller::getFan() { return this->fan; }

void Controller::setLastMeasurementTimeMs(
    const unsigned long lastMeasurementTimeMs) {
  this->lastMeasurementTimeMs = lastMeasurementTimeMs;
}

Controller::Controller(const unsigned long intervalMs,
                       peripherals::Dht22 *dht22, peripherals::Fan *fan)
    : intervalMs(intervalMs), lastMeasurementTimeMs(millis()), dht22(dht22),
      fan(fan) {}

Controller::~Controller() {}

bool Controller::IsMeasurementTimeReached() {
  return millis() - this->getLastMeasurementTimeMs() >= this->getIntervalMs();
}

Measures Controller::Measure() {
  peripherals::AirProperties airProperties = this->getDht22()->Read();
  peripherals::FanProperties fanProperties = this->getFan()->Read();

  this->setLastMeasurementTimeMs(millis());

  return Measures(airProperties, fanProperties);
}

void Controller::SetFanFrequency(float frequency) {
  this->getFan()->SetFrequency(frequency);
}

} // namespace control
