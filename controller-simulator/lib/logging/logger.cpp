#include <Arduino.h>

#include "logger.h"

namespace logging {

const unsigned long Logger::getBaudRate() const { return this->baudRate; }

Logger::Logger() : baudRate(921600) {}

Logger::~Logger() {}

void Logger::Initialize() {
  Serial.begin(this->getBaudRate());
  while (!Serial) {
  }
}

void Logger::Error(String message) {
  Serial.print("[ERROR] ");
  Serial.println(message);
}

void Logger::Warning(String message) {
  Serial.print("[WARNING] ");
  Serial.println(message);
}

void Logger::Debug(String message) {
  Serial.print("[DEBUG] ");
  Serial.println(message);
}

void Logger::Info(String message) {
  Serial.print("[INFO] ");
  Serial.println(message);
}

Logger *logger = new Logger();

} // namespace logging