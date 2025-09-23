#include "logger.h"

namespace logging {

const unsigned long Logger::getBaudRate() const { return this->baudRate; }

Logger::Logger() : baudRate(115200) { this->initialize(); }

Logger::~Logger() {}

void Logger::initialize() {
  Serial.begin(this->getBaudRate());
  while (!Serial)
    continue;
}

void Logger::Error(String message) {
  Serial.print("ERROR: ");
  Serial.println(message);
}

void Logger::Warning(String message) {
  Serial.print("WARN: ");
  Serial.println(message);
}

void Logger::Debug(String message) {
  Serial.print("DEBUG: ");
  Serial.println(message);
}

void Logger::Info(String message) {
  Serial.print("INFO: ");
  Serial.println(message);
}

Logger *logger = new Logger();

} // namespace logging
