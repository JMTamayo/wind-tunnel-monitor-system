#ifndef WIFI_H
#define WIFI_H

#include <WiFi.h>

#include "logger.h"

#include "led.h"

namespace services {

class WifiService {
private:
  const char *ssid;
  const char *password;
  const unsigned long maxRetryTimeMs;

  peripherals::Led *led;

  const char *getSsid() const;

  const char *getPassword() const;

  const unsigned long getMaxRetryTimeMs() const;

  peripherals::Led *getLed();

public:
  WifiService(const char *ssid, const char *password,
              const unsigned long maxRetryTimeMs, peripherals::Led *led);

  ~WifiService();

  void Connect();

  bool IsConnected();
};

} // namespace services

#endif // WIFI_H
