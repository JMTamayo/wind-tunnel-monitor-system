#ifndef SDM630MCT_H
#define SDM630MCT_H

#include <Arduino.h>

namespace peripherals {

class ElectricProperties {
private:
  float totalActivePower;

public:
  ElectricProperties(float totalActivePower);

  ~ElectricProperties();

  float GetTotalActivePower();
};

class SDM630MCT {
private:
  unsigned int deviceAddress;
  unsigned int totalActivePowerAddress;

  unsigned int getDeviceAddress() const;

  unsigned int getTotalActivePowerAddress() const;

  float getFloatFromData(uint16_t *data);

public:
  SDM630MCT(unsigned int rx2Pin, unsigned int tx2Pin,
            unsigned int deviceAddress, unsigned int totalActivePowerAddress);

  ~SDM630MCT();

  void Begin();

  ElectricProperties Read();
};

} // namespace peripherals

#endif // SDM630MCT_H