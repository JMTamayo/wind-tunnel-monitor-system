#include "sdm630mct.h"

namespace peripherals {

ElectricProperties::ElectricProperties(float totalActivePower)
    : totalActivePower(totalActivePower) {}

ElectricProperties::~ElectricProperties() {}

float ElectricProperties::GetTotalActivePower() {
  return this->totalActivePower;
}

unsigned int SDM630MCT::getDeviceAddress() const { return this->deviceAddress; }

unsigned int SDM630MCT::getTotalActivePowerAddress() const {
  return this->totalActivePowerAddress;
}

float SDM630MCT::getFloatFromData(uint16_t *data) {
  float value;

  uint32_t rawData = ((uint32_t)data[0] << 16) | data[1];
  memcpy(&value, &rawData, sizeof(float));

  return value;
}

SDM630MCT::SDM630MCT(unsigned int rx2Pin, unsigned int tx2Pin,
                     unsigned int deviceAddress,
                     unsigned int totalActivePowerAddress)
    : deviceAddress(deviceAddress),
      totalActivePowerAddress(totalActivePowerAddress) {
  /*
  TODO: Implement the real program to setup the device.
  Documentation:
  - Datasheet:
    https://media.adeo.com/mkp/aa1a4ec83e1fa3f4cccd74817d03b0f2/media.pdf
  - Utilities:
    https://how2electronics.com/how-to-use-modbus-rtu-with-esp32-to-read-sensor-data/
    https://www.youtube.com/watch?v=u5NQ4XxHyMw&embeds_referring_euri=https%3A%2F%2Fgemini.google.com%2F&embeds_referring_origin=https%3A%2F%2Fgemini.google.com&source_ve_path=MjM4NTE
    https://www.youtube.com/watch?v=O9ceOLX_Rgo

  We need to install and import the libraries: <HardwareSerial.h and
  <ModbusRTU.h>. Then, declare a new hardware serial and ModbusRTU. This
  utilities may be implemented as a class attributes.
  ```c++
  #include <HardwareSerial.h>
  #include <ModbusRTU.h>

  HardwareSerial serial2(2);
  ModbusRTU mb;
  ```
  */
}

SDM630MCT::~SDM630MCT() {}

void SDM630MCT::Begin() {
  /*
  TODO: Implement the real program to begin the device.

  We need to begin the serial and the ModbusRTU.
  ```c++
  serial2.begin(9600, SERIAL_8N2, this->getRx2Pin(), this->getTx2Pin());
  mb.begin(serial2);
  mb.slave(this->getDeviceAddress());
  ```
  */
}

ElectricProperties SDM630MCT::Read() {
  /*
  TODO: Implement the real program to read the device.

  Define the variables to store the data.
  ```c++
  uint8_t result;
  uint16_t data[2];

  float totalActivePower;
  ```

  Read the total active power.
  ```c++
  result = mb.readHoldingRegisters(this->getTotalActivePowerAddress(), 2);
  if (result == mb.ku8MBSuccess) {
    data[0] = mb.getResponseBuffer(0);
    data[1] = mb.getResponseBuffer(1);
    totalActivePower = this->getFloatFromData(data);
  } else {
    totalActivePower = NAN;
  }
  ```

  The following values are for testing purposes.
  */
  float totalActivePower = random(3000, 3200) / 1000.0f;
  /* --- */

  return ElectricProperties(totalActivePower);
}

} // namespace peripherals