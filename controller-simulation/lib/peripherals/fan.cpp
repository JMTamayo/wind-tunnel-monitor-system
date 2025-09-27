#include "fan.h"

namespace peripherals {

FanProperties::FanProperties(float rpm) : rpm(rpm) {}

FanProperties::~FanProperties() {}

float FanProperties::GetRpm() { return this->rpm; }

const unsigned int Fan::getControlPin() const { return this->controlPin; }

float Fan::getSetPointFrequency() const { return this->setPointFrequency; }

void Fan::setSetPointFrequency(float setPointFrequency) {
  this->setPointFrequency = setPointFrequency;
}

Fan::Fan(const unsigned int controlPin)
    : controlPin(controlPin), setPointFrequency(0.0f) {
  pinMode(this->getControlPin(), OUTPUT);
  analogWrite(this->getControlPin(), this->getSetPointFrequency());
}

Fan::~Fan() {}

void Fan::SetFrequency(float frequency) {
  float constrainedFrequency = constrain(frequency, 0.0f, 100.0f);
  unsigned int mappedFrequency =
      map(constrainedFrequency, 0.0f, 100.0f, 0, 4095);

  logging::logger->Debug("Setting fan frequency. Requested frequency: " +
                         String(constrainedFrequency) + "%.");

  this->setSetPointFrequency(constrainedFrequency);
  analogWrite(this->getControlPin(), mappedFrequency);
}

FanProperties Fan::Read() {
  /*
  TODO: Implement fan read. The following values are for testing purposes.

  The real function to read the DHT22 is:
  // TODO: Include real fan read.
  */
  float rpm = 0.0f;

  float setPointFrequency = this->getSetPointFrequency() / 100.0f;
  if (!(setPointFrequency == 0.0f)) {
    float deviation = random(-5, 5);
    rpm = max(0.0f, (float)(setPointFrequency * 1500.0 + deviation));
  }

  return FanProperties(rpm);
}

} // namespace peripherals