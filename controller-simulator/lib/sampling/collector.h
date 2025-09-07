#ifndef COLLECTOR_H
#define COLLECTOR_H

#include "time_.h"
#include "sample.h"

namespace sampling {

class Collector {
private:
  unsigned long intervalMs;
  unsigned long lastSampleTimeMs;

  services::TimeService *timeService;

  unsigned long getIntervalMs() const;

  unsigned long getLastSampleTimeMs() const;

  void setLastSampleTimeMs(unsigned long lastSampleTimeMs);

  services::TimeService *getTimeService() const;

public:
  Collector(unsigned long intervalMs, services::TimeService *timeService);

  ~Collector();

  bool IsSamplingTimeReached();

  Sample Collect();
};

} // namespace sampling

#endif // COLLECTOR_H