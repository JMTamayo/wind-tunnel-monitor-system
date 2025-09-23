#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>
#include <WiFi.h>

#include "logger.h"

namespace services {

class MqttMessage {
private:
  String topic;
  String payload;

public:
  MqttMessage(String topic, const char *payload);

  MqttMessage(String topic, String payload);

  MqttMessage(String topic, bool payload);

  MqttMessage(String topic, float payload);

  MqttMessage(String topic, int payload);

  MqttMessage(String topic, long payload);

  ~MqttMessage();

  const char *GetTopic() const;

  const char *GetPayload() const;
};

class MqttService {
private:
  const char *server;
  const unsigned int port;
  const char *user;
  const char *password;
  const char *clientId;
  const unsigned long maxRetryTimeMs;

  PubSubClient *client;

  const char *getServer() const;

  const unsigned int getPort() const;

  const char *getUser() const;

  const char *getPassword() const;

  const char *getClientId() const;

  const unsigned long getMaxRetryTimeMs() const;

  PubSubClient *getClient();

public:
  MqttService(const char *server, const unsigned int port, const char *user,
              const char *password, const char *clientId,
              const unsigned long maxRetryTimeMs, PubSubClient *client);

  ~MqttService();

  void Connect();

  bool IsConnected();

  void Publish(MqttMessage *message);

  void Subscribe(String topic);

  void Loop();
};

} // namespace services

#endif // MQTT_H
