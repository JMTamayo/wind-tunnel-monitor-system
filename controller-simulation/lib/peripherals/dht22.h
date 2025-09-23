#ifndef DHT22_H
#define DHT22_H

#include <DHT.h>

namespace peripherals {

class AirProperties {
private:
  float temperature;
  float relativeHumidity;

public:
  AirProperties(float temperature, float relativeHumidity);

  ~AirProperties();

  float GetTemperature();

  float GetRelativeHumidity();
};

class Dht22 {
private:
  DHT dht;

  DHT getDht();

public:
  Dht22(unsigned int pin);

  ~Dht22();

  AirProperties Read();
};

} // namespace peripherals

#endif // DHT22_H
