#include <Arduino.h>

#include "collector.h"
#include "logger.h"

namespace sampling {

unsigned long Collector::getIntervalMs() const { return this->intervalMs; }

unsigned long Collector::getLastSampleTimeMs() const {
  return this->lastSampleTimeMs;
}

void Collector::setLastSampleTimeMs(unsigned long lastSampleTimeMs) {
  this->lastSampleTimeMs = lastSampleTimeMs;
}

services::TimeService *Collector::getTimeService() const {
  return this->timeService;
}

Collector::Collector(unsigned long intervalMs,
                     services::TimeService *timeService)
    : intervalMs(intervalMs), lastSampleTimeMs(millis()),
      timeService(timeService) {
  this->timeService->Sync();
}

Collector::~Collector() {}

bool Collector::IsSamplingTimeReached() {
  return millis() - this->getLastSampleTimeMs() >= this->getIntervalMs();
}

Sample Collector::Collect() {
  logging::logger->Info("Collecting samples from sensors");

  float area = 1.44;
  float areaCFM = area * 10.764;

  float airVelocity = random(1000, 1200) / 100.0;
  float airVelocityCFM = airVelocity * 196.85;

  float airFlow = airVelocity * areaCFM;
  float airFlowCFM = airVelocityCFM * areaCFM;

  float airPressure = random(8400, 8600) / 100.0;
  float airTemperature = random(2400, 2600) / 100.0;
  float fanFrequency = random(45, 55) / 100.0;
  bool fanState = true;
  float turbineAxialForce = random(38000, 40000) / 100.0;
  float turbineTransverseForce = random(1800, 2000) / 100.0;
  float turbinePower = random(280, 330) / 100.0;

  String timestamp = this->getTimeService()->NowUTC();

  this->setLastSampleTimeMs(millis());

  logging::logger->Info("Sensors data collected successfully");

  return Sample(timestamp, airFlow, airFlowCFM, airPressure, airTemperature,
                airVelocity, fanFrequency, fanState, turbineAxialForce,
                turbineTransverseForce, turbinePower);
}

} // namespace sampling