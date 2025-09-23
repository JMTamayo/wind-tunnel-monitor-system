#include "mqtt.h"

namespace services {

const char *MqttMessage::GetTopic() const { return this->topic.c_str(); }

const char *MqttMessage::GetPayload() const { return this->payload.c_str(); }

MqttMessage::MqttMessage(String topic, const char *payload)
    : topic(topic), payload(String(payload)) {}

MqttMessage::MqttMessage(String topic, String payload)
    : topic(topic), payload(payload) {}

MqttMessage::MqttMessage(String topic, bool payload)
    : topic(topic), payload(payload ? "1" : "0") {}

MqttMessage::MqttMessage(String topic, float payload)
    : topic(topic), payload(String(payload)) {}

MqttMessage::MqttMessage(String topic, int payload)
    : topic(topic), payload(String(payload)) {}

MqttMessage::MqttMessage(String topic, long payload)
    : topic(topic), payload(String(payload)) {}

MqttMessage::~MqttMessage() {}

const char *MqttService::getClientId() const { return this->clientId; }

const char *MqttService::getServer() const { return this->server; }

const unsigned int MqttService::getPort() const { return this->port; }

const char *MqttService::getUser() const { return this->user; }

const char *MqttService::getPassword() const { return this->password; }

const unsigned long MqttService::getMaxRetryTimeMs() const {
  return this->maxRetryTimeMs;
}

PubSubClient *MqttService::getClient() { return this->client; }

MqttService::MqttService(const char *server, const unsigned int port,
                         const char *user, const char *password,
                         const char *clientId,
                         const unsigned long maxRetryTimeMs,
                         PubSubClient *client)
    : server(server), port(port), user(user), password(password),
      clientId(clientId), maxRetryTimeMs(maxRetryTimeMs), client(client) {
  client->setServer(this->getServer(), this->getPort());
}

MqttService::~MqttService() { delete this->client; }

void MqttService::Connect() {
  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (!this->IsConnected()) {
    if (this->getClient()->connect(this->getClientId(), this->getUser(),
                                   this->getPassword())) {
      break;
    }

    if (retryTimeMs >= this->getMaxRetryTimeMs()) {
      logging::logger->Error("Connection to MQTT server failed. Server: " +
                             String(this->getServer()));
      return;
    }

    retryTimeMs = millis() - startTimeMs;
  }

  logging::logger->Info(
      "Connected to MQTT server. Server: " + String(this->getServer()) +
      ". Client ID: " + String(this->getClientId()));
}

bool MqttService::IsConnected() { return this->getClient()->connected(); }

void MqttService::Publish(MqttMessage *message) {
  logging::logger->Debug("Publishing message to MQTT server. Topic: " +
                         String(message->GetTopic()) +
                         ". Payload: " + String(message->GetPayload()));
  this->getClient()->publish(message->GetTopic(), message->GetPayload());
}

void MqttService::Subscribe(String topic) {
  this->getClient()->subscribe(topic.c_str());
  logging::logger->Debug("Subscribed to topic from MQTT server: " + topic);
}

void MqttService::Loop() { this->getClient()->loop(); }

} // namespace services
