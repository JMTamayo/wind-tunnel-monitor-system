#include "fan.h"

namespace peripherals {

FanProperties::FanProperties(float rpm, float frequencySetPoint,
                             float totalActivePower)
    : rpm(rpm), frequencySetPoint(frequencySetPoint),
      totalActivePower(totalActivePower) {}

FanProperties::~FanProperties() {}

float FanProperties::GetRpm() { return this->rpm; }

float FanProperties::GetFrequencySetPoint() { return this->frequencySetPoint; }

float FanProperties::GetTotalActivePower() { return this->totalActivePower; }

const unsigned int Fan::getControlPin() const { return this->controlPin; }

void Fan::setFrequencySetPoint(float frequencySetPoint) {
  this->frequencySetPoint = frequencySetPoint;
}

peripherals::SDM630MCT *Fan::getSdm630mct() { return this->sdm630mct; }

Fan::Fan(const unsigned int controlPin, peripherals::SDM630MCT *sdm630mct)
    : controlPin(controlPin), frequencySetPoint(0.0f), sdm630mct(sdm630mct) {
  /*
  TODO: Implement the real program to setup the fan.
  Currently, we are using a dummy fan using a LED. It is pending to define the
  real fan to implement the real program.

  The following values are for testing purposes.
  */
  pinMode(this->getControlPin(), OUTPUT);
  analogWrite(this->getControlPin(), this->GetFrequencySetPoint());
  /* --- */
}

Fan::~Fan() {}

void Fan::Begin() {
  /*
  TODO: Implement the real program to begin the device.

  We need to begin the elements of the fan as follows:
  ```c++
  this->getSdm630mct()->Begin();
  ```
  */
}

float Fan::GetFrequencySetPoint() const { return this->frequencySetPoint; }

void Fan::SetFrequency(float frequency) {
  /*
  TODO: Implement the real program to set the fan frequency.
  It is pending to define the real fan to implement the real program.

  The following values are for testing purposes.
  */
  float constrainedFrequency = constrain(frequency, 0.0f, 100.0f);
  unsigned int mappedFrequency =
      map(constrainedFrequency, 0.0f, 100.0f, 0, 255);

  logging::logger->Debug("Setting fan frequency. Requested frequency: " +
                         String(constrainedFrequency) + "%.");

  this->setFrequencySetPoint(constrainedFrequency);
  analogWrite(this->getControlPin(), mappedFrequency);
  /* --- */
}

FanProperties Fan::Read() {
  /*
  TODO: Implement the real program to read the fan.
  It is pending to define the real fan to implement the real program.

  The following values are for testing purposes.
  */
  float rpm = 0.0f;

  float frequencySetPoint = this->GetFrequencySetPoint();
  if (!(frequencySetPoint == 0.0f)) {
    float deviation = random(-5, 5);
    rpm = max(0.0f, (float)(frequencySetPoint * 1500.0 / 100.0 + deviation));
  }

  peripherals::ElectricProperties electricProperties =
      this->getSdm630mct()->Read();
  float totalActivePower = electricProperties.GetTotalActivePower();

  return FanProperties(rpm, frequencySetPoint, totalActivePower);
  /* --- */
}

} // namespace peripherals