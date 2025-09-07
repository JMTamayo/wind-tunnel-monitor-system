#include <WiFi.h>

#include "led.h"
#include "logger.h"
#include "wifi_.h"

namespace services {

const char *WifiService::getSsid() const { return this->ssid; }

const char *WifiService::getPassword() const { return this->password; }

unsigned long WifiService::getMaxRetryTimeMs() const {
  return this->maxRetryTimeMs;
}

peripherals::Led *WifiService::getIndicator() { return this->indicator; }

WifiService::WifiService(const char *ssid, const char *password,
                         unsigned long maxRetryTimeMs,
                         peripherals::Led *indicator)
    : ssid(ssid), password(password), maxRetryTimeMs(maxRetryTimeMs),
      indicator(indicator) {}

WifiService::~WifiService() { delete this->indicator; }

void WifiService::Connect() {
  this->indicator->On();

  WiFi.begin(this->getSsid(), this->getPassword());

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  logging::logger->Info("Connecting to WiFi network: " +
                        String(this->getSsid()));

  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (!this->IsConnected()) {
    if (retryTimeMs >= this->getMaxRetryTimeMs()) {
      logging::logger->Error(
          "Connection to WiFi network failed. Restarting the device.");
      ESP.restart();
    }

    delay(100);
    retryTimeMs = millis() - startTimeMs;

    logging::logger->Warning(
        "WiFi connection not established. Status: " + String(WiFi.status()) +
        ". Retry time [ms]: " + String(retryTimeMs));
  }

  logging::logger->Info("WiFi connection successfully established");
  logging::logger->Info("IP: " + String(WiFi.localIP().toString().c_str()));

  this->indicator->Off();
}

bool WifiService::IsConnected() { return WiFi.status() == WL_CONNECTED; }

} // namespace services