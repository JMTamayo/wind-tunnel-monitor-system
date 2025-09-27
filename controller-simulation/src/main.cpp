#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "parameters.h"
#include "secrets.h"

#include "logger.h"

#include "controller.h"
#include "dht22.h"
#include "fan.h"

#include "mqtt.h"
#include "wifi_.h"

TaskHandle_t ServerHandlingTask;
TaskHandle_t ControlHandlingTask;

QueueHandle_t MqttPublishingEventQueue;
QueueHandle_t MqttSubscriptionsEventQueue;

String TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT =
    String(MQTT_TOPIC_BASE) +
    String(MQTT_TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT);
String TOPIC_FAN_RPM = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_FAN_RPM);

String TOPIC_MEASUREMENTS_AIR_TEMPERATURE =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_AIR_TEMPERATURE);
String TOPIC_MEASUREMENTS_AIR_RELATIVE_HUMIDITY =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_AIR_RELATIVE_HUMIDITY);

void mqttSubscriptionCallback(char *topic, byte *payload, unsigned int length) {
  String topicStr = topic;
  String payloadString = (char *)payload;
  payloadString.remove(length);

  services::MqttMessage *mqttMessage =
      new services::MqttMessage(topicStr, payloadString);

  if (topicStr == TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT) {
    logging::logger->Debug("Received message from MQTT server. Topic: " +
                           topicStr + ". Payload: " + payloadString);
    xQueueSend(MqttSubscriptionsEventQueue, &mqttMessage, pdMS_TO_TICKS(100));

  } else {
    logging::logger->Error("Received invalid message from MQTT server or "
                           "message not supported. Topic: " +
                           topicStr + ". Payload: " + payloadString);
  }
}

void serverHandling(void *parameter) {
  services::WifiService *wifi = new services::WifiService(
      WIFI_SSID, WIFI_PASSWORD, WIFI_MAX_RETRY_TIME_MILLISECONDS);

  WiFiClient *espClient = new WiFiClient();
  PubSubClient *mqttClient = new PubSubClient(*espClient);
  mqttClient->setCallback(mqttSubscriptionCallback);

  services::MqttService *mqtt = new services::MqttService(
      MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD, MQTT_CLIENT_ID,
      MQTT_MAX_RETRY_TIME_MILLISECONDS, mqttClient);
  services::MqttMessage *mqttMessage;

  while (true) {
    if (!wifi->IsConnected()) {
      wifi->Connect();

    } else if (!mqtt->IsConnected()) {
      mqtt->Connect();
      mqtt->Subscribe(TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT);

    } else {
      mqtt->Loop();

      while (xQueueReceive(MqttPublishingEventQueue, &mqttMessage,
                           pdMS_TO_TICKS(10)) == pdPASS) {
        mqtt->Publish(mqttMessage);
        delete mqttMessage;
      }
    }
  }
}

void controlHandling(void *parameter) {
  peripherals::Dht22 *dht22 = new peripherals::Dht22(DHT22_PIN);

  peripherals::Fan *fan = new peripherals::Fan(FAN_CONTROL_PIN);

  control::Controller *controller =
      new control::Controller(MEASURING_INTERVAL_MILLISECONDS, dht22, fan);

  services::MqttMessage *mqttMessage;

  while (true) {
    if (xQueueReceive(MqttSubscriptionsEventQueue, &mqttMessage,
                      pdMS_TO_TICKS(10)) == pdPASS) {
      if (String(mqttMessage->GetTopic()) ==
          TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT) {
        float fanFrequency = String(mqttMessage->GetPayload()).toFloat();
        controller->SetFanFrequency(fanFrequency);
      }
      delete mqttMessage;
    }

    if (controller->IsMeasurementTimeReached()) {
      control::Measures measure = controller->Measure();

      float airTemperature = measure.GetAirProperties().GetTemperature();
      if (!isnan(airTemperature)) {
        services::MqttMessage *airTemperatureMessage =
            new services::MqttMessage(TOPIC_MEASUREMENTS_AIR_TEMPERATURE,
                                      airTemperature);
        xQueueSend(MqttPublishingEventQueue, &airTemperatureMessage,
                   pdMS_TO_TICKS(10));
      }

      float airRelativeHumidity =
          measure.GetAirProperties().GetRelativeHumidity();
      if (!isnan(airRelativeHumidity)) {
        services::MqttMessage *airRelativeHumidityMessage =
            new services::MqttMessage(TOPIC_MEASUREMENTS_AIR_RELATIVE_HUMIDITY,
                                      airRelativeHumidity);
        xQueueSend(MqttPublishingEventQueue, &airRelativeHumidityMessage,
                   pdMS_TO_TICKS(10));
      }

      float fanRpm = measure.GetFanProperties().GetRpm();
      services::MqttMessage *fanRpmMessage =
          new services::MqttMessage(TOPIC_FAN_RPM, fanRpm);
      xQueueSend(MqttPublishingEventQueue, &fanRpmMessage, pdMS_TO_TICKS(10));
    }
  }
}

void setup() {
  xTaskCreatePinnedToCore(serverHandling, "serverHandling", 10000, NULL, 1,
                          &ServerHandlingTask, 0);
  xTaskCreatePinnedToCore(controlHandling, "controlHandling", 10000, NULL, 1,
                          &ControlHandlingTask, 1);

  MqttPublishingEventQueue = xQueueCreate(3, sizeof(services::MqttMessage *));
  MqttSubscriptionsEventQueue =
      xQueueCreate(1, sizeof(services::MqttMessage *));
}

void loop() {}
