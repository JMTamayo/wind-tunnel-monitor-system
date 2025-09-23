#ifndef WIFI_H
#define WIFI_H

#include <WiFi.h>

#include "logger.h"

namespace services {

class WifiService {
private:
  const char *ssid;
  const char *password;
  const unsigned long maxRetryTimeMs;

  const char *getSsid() const;

  const char *getPassword() const;

  const unsigned long getMaxRetryTimeMs() const;

public:
  WifiService(const char *ssid, const char *password,
              const unsigned long maxRetryTimeMs);

  ~WifiService();

  void Connect();

  bool IsConnected();
};

} // namespace services

#endif // WIFI_H
