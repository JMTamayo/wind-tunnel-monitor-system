#include "MPXV7002DP_ADC1115.h"

namespace peripherals {

DynamicPressure::DynamicPressure(float pressure, float airVelocity)
    : pressure(pressure), airVelocity(airVelocity) {}

DynamicPressure::~DynamicPressure() {}

float DynamicPressure::GetPressure() { return this->pressure; }

float DynamicPressure::GetAirVelocity() { return this->airVelocity; }

float MPXV7002DP_ADC1115::getVoltageOffset() { return this->voltageOffset; }

float MPXV7002DP_ADC1115::getVoltageSensitivity() {
  return this->voltageSensitivity;
}

unsigned int MPXV7002DP_ADC1115::getChannel() { return this->channel; }

MPXV7002DP_ADC1115::MPXV7002DP_ADC1115(unsigned int channel,
                                       float voltageOffset,
                                       float voltageSensitivity)
    : channel(channel), voltageOffset(voltageOffset),
      voltageSensitivity(voltageSensitivity) {
  /*
  TODO: Implement the real program to setup the sensor.
  Documentation:
  - Datasheet:
    https://co.mouser.com/datasheet/3/118/1/MPXV7002.pdf

  We need to install and import the library: <Adafruit_ADS1X15.h>. Then,
  declare a new instance of the class Adafruit_ADS1115:
  ```c++
  #include <Adafruit_ADS1X15.h>
  Adafruit_ADS1115 ads;
  ```
  */
}

MPXV7002DP_ADC1115::~MPXV7002DP_ADC1115() {}

void MPXV7002DP_ADC1115::Begin() {
  /*
  TODO: Implement the real program to begin the sensor.

  We need to begin the reading of the sensor as follows:
  ```c++
  if (!ads.begin()) {
    Serial.println("Failed to initialize the ADC.");
    while (1);
  }
  ```
  */
}

DynamicPressure MPXV7002DP_ADC1115::Read(float airDensity) {
  /*
  TODO: Implement the real program to read the sensor.

  Read ADC value in single mode and compute the voltage.
  ```c++
  int16_t adcValue = ads.readADC_SingleEnded(this->getChannel());
  float voltage = ads.computeVolts(adcValue);
  ```

  The following values are for testing purposes.
  */
  float voltage = random(2550, 2560) / 1000.0f;
  /* --- */

  float pressure =
      (voltage - this->getVoltageOffset()) / this->getVoltageSensitivity();
  if (pressure < 0.0f)
    pressure = 0.0f;

  float airVelocity = sqrt(2.0f * (1000 * pressure) / airDensity);

  return DynamicPressure(pressure, airVelocity);
}

} // namespace peripherals