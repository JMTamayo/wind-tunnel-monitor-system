#include "dht22.h"

namespace peripherals {

AirProperties::AirProperties(float temperature, float relativeHumidity)
    : temperature(temperature), relativeHumidity(relativeHumidity) {}

AirProperties::~AirProperties() {}

float AirProperties::GetTemperature() { return this->temperature; }

float AirProperties::GetRelativeHumidity() { return this->relativeHumidity; }

DHT Dht22::getDht() { return this->dht; }

Dht22::Dht22(const unsigned int pin) : dht(pin, DHT22) {}

Dht22::~Dht22() {}

AirProperties Dht22::Read() {
  /*
  TODO: Implement DHT22 read. The following values are for testing purposes.

  The real function to read the DHT22 is:
  return AirProperties(this->dht.readTemperature(), this->dht.readHumidity());
  */
  return AirProperties(random(240, 260) / 10.0, random(500, 550) / 10.0);
}

} // namespace peripherals
