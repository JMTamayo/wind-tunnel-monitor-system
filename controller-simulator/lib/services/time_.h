#ifndef TIME_H
#define TIME_H

namespace services {

class TimeService {
private:
  const char *ntpServer;
  const int timeZoneOffset;
  const int daylightOffset;
  const unsigned long maxRetryTimeMs;
  unsigned long lastSyncTimeMs;

  const char *getNtpServer() const;

  const int getTimeZoneOffset() const;

  int getDaylightOffset() const;

  unsigned long getMaxRetryTimeMs() const;

  unsigned long getLastSyncTimeMs() const;

  void setLastSyncTimeMs(unsigned long lastSyncTimeMs);

public:
  TimeService(const char *ntpServer, const int timeZoneOffset,
              const int daylightOffset, const unsigned long maxRetryTimeMs);

  ~TimeService();

  void Sync();

  String NowUTC();
};

} // namespace services

#endif // TIME_H