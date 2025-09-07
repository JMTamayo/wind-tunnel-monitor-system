#include <Arduino.h>

#include "led.h"

namespace peripherals {

unsigned int Led::getPin() const { return this->pin; }

Led::Led(unsigned int pin) : pin(pin) {
  pinMode(this->getPin(), OUTPUT);
  this->Off();
}

Led::~Led() {}

void Led::Off() { digitalWrite(this->getPin(), LOW); }

void Led::On() { digitalWrite(this->getPin(), HIGH); }

} // namespace peripherals