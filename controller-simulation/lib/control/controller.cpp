#include "controller.h"

namespace control {

Measures::Measures(peripherals::AirProperties airProperties)
    : airProperties(airProperties) {}

Measures::~Measures() {}

peripherals::AirProperties Measures::GetAirProperties() {
  return this->airProperties;
}

const unsigned long Controller::getIntervalMs() const {
  return this->intervalMs;
}

unsigned long Controller::getLastMeasurementTimeMs() const {
  return this->lastMeasurementTimeMs;
}

peripherals::Dht22 *Controller::getDht22() { return this->dht22; }

void Controller::setLastMeasurementTimeMs(
    const unsigned long lastMeasurementTimeMs) {
  this->lastMeasurementTimeMs = lastMeasurementTimeMs;
}

Controller::Controller(const unsigned long intervalMs,
                       peripherals::Dht22 *dht22)
    : intervalMs(intervalMs), lastMeasurementTimeMs(millis()), dht22(dht22) {}

Controller::~Controller() {}

bool Controller::IsMeasurementTimeReached() {
  return millis() - this->getLastMeasurementTimeMs() >= this->getIntervalMs();
}

Measures Controller::Measure() {
  peripherals::AirProperties airProperties = this->getDht22()->Read();

  this->setLastMeasurementTimeMs(millis());

  return Measures(airProperties);
}

} // namespace control
