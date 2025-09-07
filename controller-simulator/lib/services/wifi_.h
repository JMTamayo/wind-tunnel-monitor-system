#ifndef WIFI_H
#define WIFI_H

#include "led.h"

namespace services {

class WifiService {
private:
  const char *ssid;
  const char *password;
  const unsigned long maxRetryTimeMs;

  peripherals::Led *indicator;

  const char *getSsid() const;

  const char *getPassword() const;

  unsigned long getMaxRetryTimeMs() const;

  peripherals::Led *getIndicator();

public:
  WifiService(const char *ssid, const char *password,
              unsigned long maxRetryTimeMs, peripherals::Led *indicator);

  ~WifiService();

  void Connect();

  bool IsConnected();
};

} // namespace services

#endif // WIFI_H