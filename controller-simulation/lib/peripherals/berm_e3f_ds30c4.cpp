#include "berm_e3f_ds30c4.h"

namespace peripherals {

RotationalFrequency::RotationalFrequency(unsigned long pulseCount,
                                         unsigned long sampleTimeSeconds,
                                         float rotationalFrequencyHz,
                                         float rotationalFrequencyRpm)
    : pulseCount(pulseCount), sampleTimeSeconds(sampleTimeSeconds),
      rotationalFrequencyHz(rotationalFrequencyHz),
      rotationalFrequencyRpm(rotationalFrequencyRpm) {}

RotationalFrequency::~RotationalFrequency() {}

unsigned long RotationalFrequency::GetPulseCount() { return this->pulseCount; }

unsigned long RotationalFrequency::GetSampleTimeSeconds() {
  return this->sampleTimeSeconds;
}

float RotationalFrequency::GetRotationalFrequencyHz() {
  return this->rotationalFrequencyHz;
}

float RotationalFrequency::GetRotationalFrequencyRpm() {
  return this->rotationalFrequencyRpm;
}

volatile unsigned long pulseCounter = 0;

void IRAM_ATTR incrementPulseCount() { pulseCounter++; }

const unsigned int BermE3fDs30c4::getPin() const { return this->pin; }

unsigned long BermE3fDs30c4::getLastMeasurementTimeMs() const {
  return this->lastMeasurementTimeMs;
}

void BermE3fDs30c4::setLastMeasurementTimeMs(
    unsigned long lastMeasurementTimeMs) {
  this->lastMeasurementTimeMs = lastMeasurementTimeMs;
}

BermE3fDs30c4::BermE3fDs30c4(const unsigned int pin) : pin(pin) {
  pinMode(this->getPin(), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), incrementPulseCount, RISING);
}

BermE3fDs30c4::~BermE3fDs30c4() {}

void BermE3fDs30c4::Restart() {
  pulseCounter = 0;
  this->setLastMeasurementTimeMs(millis());
}

RotationalFrequency BermE3fDs30c4::Read(unsigned int pulseCountDivider) {
  /*
  unsigned long pulseCount = pulseCounter;
  */

  // The following values are for testing purposes. You should delete this when
  // the real prototype is ready.
  unsigned long pulseCount = random(100, 110);
  unsigned long sampleTimeSeconds =
      (millis() - this->getLastMeasurementTimeMs()) / 1000;

  float rotationalFrequencyHz =
      (float)(pulseCount / sampleTimeSeconds) / (float)pulseCountDivider;
  float rotationalFrequencyRpm = rotationalFrequencyHz * 60;

  this->Restart();

  return RotationalFrequency(pulseCount, sampleTimeSeconds,
                             rotationalFrequencyHz, rotationalFrequencyRpm);
}

} // namespace peripherals