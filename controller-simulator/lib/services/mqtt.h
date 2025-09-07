#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>

#include "led.h"
#include "sample.h"

namespace services {

class MqttService {
private:
  const char *server;
  const char *user;
  const char *password;
  const char *clientId;
  const unsigned long maxRetryTimeMs;

  const char *topicBase;
  const char *topicAirFlow;
  const char *topicAirFlowCFM;
  const char *topicAirPressure;
  const char *topicAirTemperature;
  const char *topicAirVelocity;
  const char *topicFanFrequency;
  const char *topicFanFrequencySetPoint;
  const char *topicFanState;
  const char *topicTimestamp;
  const char *topicTurbineAxialForce;
  const char *topicTurbineTransverseForce;
  const char *topicTurbinePower;

  peripherals::Led *indicator;

  PubSubClient *client;

  const char *getServer() const;

  const char *getUser() const;

  const char *getPassword() const;

  const char *getClientId() const;

  unsigned long getMaxRetryTimeMs() const;

  const char *getTopicBase() const;

  const char *getTopicAirFlow() const;

  const char *getTopicAirFlowCFM() const;

  const char *getTopicAirPressure() const;

  const char *getTopicAirTemperature() const;

  const char *getTopicAirVelocity() const;

  const char *getTopicFanFrequency() const;

  const char *getTopicFanFrequencySetPoint() const;

  const char *getTopicFanState() const;

  const char *getTopicTimestamp() const;

  const char *getTopicTurbineAxialForce() const;

  const char *getTopicTurbineTransverseForce() const;

  const char *getTopicTurbinePower() const;

  peripherals::Led *getIndicator();

  PubSubClient *getClient();

public:
  MqttService(const char *server, const char *user, const char *password,
              const char *clientId, const unsigned long maxRetryTimeMs,
              const char *topicBase, const char *topicAirFlow,
              const char *topicAirFlowCFM, const char *topicAirPressure,
              const char *topicAirTemperature, const char *topicAirVelocity,
              const char *topicFanFrequency,
              const char *topicFanFrequencySetPoint, const char *topicFanState,
              const char *topicTimestamp, const char *topicTurbineAxialForce,
              const char *topicTurbineTransverseForce,
              const char *topicTurbinePower, peripherals::Led *indicator,
              PubSubClient *client);

  ~MqttService();

  bool IsConnected();

  void Connect();

  void Loop();

  void Publish(sampling::Sample sample);
};

} // namespace services

#endif // MQTT_H