#include <PubSubClient.h>
#include <WiFi.h>

#include "logger.h"
#include "mqtt.h"

namespace services {

const char *MqttService::getClientId() const { return this->clientId; }

const char *MqttService::getServer() const { return this->server; }

const char *MqttService::getUser() const { return this->user; }

const char *MqttService::getPassword() const { return this->password; }

unsigned long MqttService::getMaxRetryTimeMs() const {
  return this->maxRetryTimeMs;
}

const char *MqttService::getTopicBase() const { return this->topicBase; }

const char *MqttService::getTopicAirFlow() const { return this->topicAirFlow; }

const char *MqttService::getTopicAirFlowCFM() const {
  return this->topicAirFlowCFM;
}

const char *MqttService::getTopicAirPressure() const {
  return this->topicAirPressure;
}

const char *MqttService::getTopicAirTemperature() const {
  return this->topicAirTemperature;
}

const char *MqttService::getTopicAirVelocity() const {
  return this->topicAirVelocity;
}

const char *MqttService::getTopicFanFrequency() const {
  return this->topicFanFrequency;
}

const char *MqttService::getTopicFanFrequencySetPoint() const {
  return this->topicFanFrequencySetPoint;
}

const char *MqttService::getTopicFanState() const {
  return this->topicFanState;
}

const char *MqttService::getTopicTimestamp() const {
  return this->topicTimestamp;
}

const char *MqttService::getTopicTurbineAxialForce() const {
  return this->topicTurbineAxialForce;
}

const char *MqttService::getTopicTurbineTransverseForce() const {
  return this->topicTurbineTransverseForce;
}

const char *MqttService::getTopicTurbinePower() const {
  return this->topicTurbinePower;
}

peripherals::Led *MqttService::getIndicator() { return this->indicator; }

PubSubClient *MqttService::getClient() { return this->client; }

MqttService::MqttService(
    const char *server, const char *user, const char *password,
    const char *clientId, const unsigned long maxRetryTimeMs,
    const char *topicBase, const char *topicAirFlow,
    const char *topicAirFlowCFM, const char *topicAirPressure,
    const char *topicAirTemperature, const char *topicAirVelocity,
    const char *topicFanFrequency, const char *topicFanFrequencySetPoint,
    const char *topicFanState, const char *topicTimestamp,
    const char *topicTurbineAxialForce, const char *topicTurbineTransverseForce,
    const char *topicTurbinePower, peripherals::Led *indicator,
    PubSubClient *client)
    : server(server), user(user), password(password), clientId(clientId),
      maxRetryTimeMs(maxRetryTimeMs), topicBase(topicBase),
      topicAirFlow(topicAirFlow), topicAirFlowCFM(topicAirFlowCFM),
      topicAirPressure(topicAirPressure),
      topicAirTemperature(topicAirTemperature),
      topicAirVelocity(topicAirVelocity), topicFanFrequency(topicFanFrequency),
      topicFanFrequencySetPoint(topicFanFrequencySetPoint),
      topicFanState(topicFanState), topicTimestamp(topicTimestamp),
      topicTurbineAxialForce(topicTurbineAxialForce),
      topicTurbineTransverseForce(topicTurbineTransverseForce),
      topicTurbinePower(topicTurbinePower), indicator(indicator),
      client(client) {
  client->setServer(this->getServer(), 1883);
}

MqttService::~MqttService() { delete this->client; }

bool MqttService::IsConnected() { return this->client->connected(); }

void MqttService::Connect() {
  this->getIndicator()->On();

  logging::logger->Info("Connecting to MQTT server: " +
                        String(this->getServer()));

  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (!this->IsConnected()) {
    if (this->client->connect(this->getClientId(), this->getUser(),
                              this->getPassword())) {
      break;
    }

    if (retryTimeMs >= this->getMaxRetryTimeMs()) {
      logging::logger->Error(
          "Connection to MQTT server failed. Restarting the device.");
      ESP.restart();
    }

    logging::logger->Warning(
        "MQTT connection to server not established. Status: " +
        String(this->client->state()) +
        ". Retry time [ms]: " + String(retryTimeMs));

    delay(100);
    retryTimeMs = millis() - startTimeMs;
  }

  logging::logger->Info("MQTT connection to server successfully established");

  this->getIndicator()->Off();
}

void MqttService::Loop() { this->client->loop(); }

void MqttService::Publish(sampling::Sample sample) {
  logging::logger->Info("Publishing sample to MQTT server");

  const String topicBase = String(this->getTopicBase()) + "/";

  this->getClient()->publish(
      (topicBase + String(this->getTopicTimestamp())).c_str(),
      sample.GetTimestamp().c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicAirFlow())).c_str(),
      String(sample.GetAirFlow()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicAirFlowCFM())).c_str(),
      String(sample.GetAirFlowCFM()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicAirPressure())).c_str(),
      String(sample.GetAirPressure()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicAirTemperature())).c_str(),
      String(sample.GetAirTemperature()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicAirVelocity())).c_str(),
      String(sample.GetAirVelocity()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicFanFrequency())).c_str(),
      String(sample.GetFanFrequency()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicFanState())).c_str(),
      String(sample.GetFanState()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicTurbineAxialForce())).c_str(),
      String(sample.GetTurbineAxialForce()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicTurbineTransverseForce())).c_str(),
      String(sample.GetTurbineTransverseForce()).c_str());

  this->getClient()->publish(
      (topicBase + String(this->getTopicTurbinePower())).c_str(),
      String(sample.GetTurbinePower()).c_str());

  logging::logger->Info("Sample published successfully");
}

} // namespace services