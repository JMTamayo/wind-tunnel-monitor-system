#include "fan.h"

namespace peripherals {

FanProperties::FanProperties(float rpm, float frequencySetPoint)
    : rpm(rpm), frequencySetPoint(frequencySetPoint) {}

FanProperties::~FanProperties() {}

float FanProperties::GetRpm() { return this->rpm; }

float FanProperties::GetFrequencySetPoint() { return this->frequencySetPoint; }

const unsigned int Fan::getControlPin() const { return this->controlPin; }

void Fan::setFrequencySetPoint(float frequencySetPoint) {
  this->frequencySetPoint = frequencySetPoint;
}

Fan::Fan(const unsigned int controlPin)
    : controlPin(controlPin), frequencySetPoint(0.0f) {
  pinMode(this->getControlPin(), OUTPUT);
  analogWrite(this->getControlPin(), this->GetFrequencySetPoint());
}

Fan::~Fan() {}

float Fan::GetFrequencySetPoint() const { return this->frequencySetPoint; }

void Fan::SetFrequency(float frequency) {
  float constrainedFrequency = constrain(frequency, 0.0f, 100.0f);
  unsigned int mappedFrequency =
      map(constrainedFrequency, 0.0f, 100.0f, 0, 255);

  logging::logger->Debug("Setting fan frequency. Requested frequency: " +
                         String(constrainedFrequency) + "%.");

  this->setFrequencySetPoint(constrainedFrequency);
  analogWrite(this->getControlPin(), mappedFrequency);
}

FanProperties Fan::Read() {
  /*
  TODO: Implement fan read. The following values are for testing purposes.

  The real function to read the DHT22 is:
  // TODO: Include real fan read.
  */
  float rpm = 0.0f;

  float frequencySetPoint = this->GetFrequencySetPoint();
  if (!(frequencySetPoint == 0.0f)) {
    float deviation = random(-5, 5);
    rpm = max(0.0f, (float)(frequencySetPoint * 1500.0 / 100.0 + deviation));
  }

  return FanProperties(rpm, frequencySetPoint);
}

} // namespace peripherals