#include "wifi_.h"

namespace services {

const char *WifiService::getSsid() const { return this->ssid; }

const char *WifiService::getPassword() const { return this->password; }

const unsigned long WifiService::getMaxRetryTimeMs() const {
  return this->maxRetryTimeMs;
}

peripherals::Led *WifiService::getLed() { return this->led; }

WifiService::WifiService(const char *ssid, const char *password,
                         const unsigned long maxRetryTimeMs,
                         peripherals::Led *led)
    : ssid(ssid), password(password), maxRetryTimeMs(maxRetryTimeMs), led(led) {
}

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

    vTaskDelay(pdMS_TO_TICKS(100));
    retryTimeMs = millis() - startTimeMs;
  }

  logging::logger->Info(
      "Connected to WiFi network. SSID: " + String(this->getSsid()) +
      ". IP: " + String(WiFi.localIP().toString().c_str()));
}

bool WifiService::IsConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    this->getLed()->Low();
    return true;
  } else {
    this->getLed()->High();
    return false;
  }
}

} // namespace services
