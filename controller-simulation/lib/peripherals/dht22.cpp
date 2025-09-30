#include "DHT22.h"

namespace peripherals {

AirProperties::AirProperties(float temperature, float relativeHumidity)
    : temperature(temperature), relativeHumidity(relativeHumidity) {}

AirProperties::~AirProperties() {}

float AirProperties::GetTemperature() { return this->temperature; }

float AirProperties::GetRelativeHumidity() { return this->relativeHumidity; }

DHT Dht22::getDht() { return this->dht; }

Dht22::Dht22(const unsigned int pin) : dht(pin, DHT22) {
  /*
  TODO: Implement the real program to setup the sensor.
  Documentation:
  - Datasheet:
    https://www.alldatasheet.com/datasheet-pdf/view/1132459/ETC2/DHT22.html
  */
}

Dht22::~Dht22() {}

void Dht22::Begin() {
  /*
  TODO: Implement the real program to begin the sensor.

  ```c++
  this->dht.begin();
  ```
  */
}

AirProperties Dht22::Read() {
  /*
  TODO: Implement the real program to read the sensor.

  Read the temperature and humidity.
  ```c++
  float temperature = this->getDht()->readTemperature();
  float relativeHumidity = this->getDht()->readHumidity();
  ```

  The following values are for testing purposes.
  */
  float temperature = random(240, 260) / 10.0;
  float relativeHumidity = random(500, 550) / 10.0;
  /* --- */

  return AirProperties(temperature, relativeHumidity);
}

} // namespace peripherals
