#include <Arduino.h>
#include <time.h>

#include "logger.h"

#include "time_.h"

namespace services {

const char *TimeService::getNtpServer() const { return this->ntpServer; }

const int TimeService::getTimeZoneOffset() const {
  return this->timeZoneOffset;
}

int TimeService::getDaylightOffset() const { return this->daylightOffset; }

unsigned long TimeService::getMaxRetryTimeMs() const {
  return this->maxRetryTimeMs;
}

unsigned long TimeService::getLastSyncTimeMs() const {
  return this->lastSyncTimeMs;
}

void TimeService::setLastSyncTimeMs(unsigned long lastSyncTimeMs) {
  this->lastSyncTimeMs = lastSyncTimeMs;
}

TimeService::TimeService(const char *ntpServer, const int timeZoneOffset,
                         const int daylightOffset,
                         const unsigned long maxRetryTimeMs)
    : ntpServer(ntpServer), timeZoneOffset(timeZoneOffset),
      daylightOffset(daylightOffset), maxRetryTimeMs(maxRetryTimeMs),
      lastSyncTimeMs(0) {}

TimeService::~TimeService() {}

void TimeService::Sync() {
  logging::logger->Info("Syncing time with NTP server: " +
                        String(this->getNtpServer()));

  configTime(this->getTimeZoneOffset(), this->getDaylightOffset(),
             this->getNtpServer());
  this->setLastSyncTimeMs(millis());

  logging::logger->Info("Time synced successfully");
}

String TimeService::NowUTC() {
  struct tm timeinfo;

  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (!getLocalTime(&timeinfo)) {
    logging::logger->Warning("Time sync failed. Retry time [ms]: " +
                             String(retryTimeMs));

    if (retryTimeMs >= this->getMaxRetryTimeMs()) {
      logging::logger->Error("Time sync failed. Restarting the device.");
      ESP.restart();
    }

    this->Sync();

    retryTimeMs = millis() - startTimeMs;
  }

  long currentMillis = millis();
  long ms = (currentMillis - this->getLastSyncTimeMs()) % 1000;

  char timeString[20];
  strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%S", &timeinfo);

  char timeStringMillis[25];
  sprintf(timeStringMillis, "%s.%03ldZ", timeString, ms);

  return String(timeStringMillis);
}

} // namespace services