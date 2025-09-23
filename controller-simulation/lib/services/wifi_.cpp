#include "wifi_.h"

namespace services {

const char *WifiService::getSsid() const { return this->ssid; }

const char *WifiService::getPassword() const { return this->password; }

const unsigned long WifiService::getMaxRetryTimeMs() const {
  return this->maxRetryTimeMs;
}

WifiService::WifiService(const char *ssid, const char *password,
                         const unsigned long maxRetryTimeMs)
    : ssid(ssid), password(password), maxRetryTimeMs(maxRetryTimeMs) {}

WifiService::~WifiService() {}

void WifiService::Connect() {
  WiFi.begin(this->getSsid(), this->getPassword());

  WiFi.mode(WIFI_STA);

  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (!this->IsConnected()) {
    if (retryTimeMs >= this->getMaxRetryTimeMs()) {
      logging::logger->Error("Connection to WiFi network failed. SSID: " +
                             String(this->getSsid()));
      return;
    }

    retryTimeMs = millis() - startTimeMs;
  }

  logging::logger->Info(
      "Connected to WiFi network. SSID: " + String(this->getSsid()) +
      ". IP: " + String(WiFi.localIP().toString().c_str()));
}

bool WifiService::IsConnected() { return WiFi.status() == WL_CONNECTED; }

} // namespace services
