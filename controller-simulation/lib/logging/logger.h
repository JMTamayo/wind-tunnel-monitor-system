#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

namespace logging {

class Logger {
private:
  const unsigned long baudRate;

  const unsigned long getBaudRate() const;

public:
  Logger();

  ~Logger();

  void initialize();

  void Error(String message);

  void Warning(String message);

  void Debug(String message);

  void Info(String message);
};

extern Logger *logger;

} // namespace logging

#endif // LOGGER_H
