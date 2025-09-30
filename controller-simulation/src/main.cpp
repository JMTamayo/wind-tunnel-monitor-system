#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "parameters.h"
#include "secrets.h"

#include "logger.h"

#include "controller.h"

#include "berm_e3f_ds30c4.h"
#include "dht22.h"
#include "fan.h"
#include "mpxv7002dp_adc1115.h"

#include "mqtt.h"
#include "wifi_.h"

TaskHandle_t ServerHandlingTask;
TaskHandle_t ControlHandlingTask;

QueueHandle_t MqttPublishingEventQueue;
QueueHandle_t MqttSubscriptionsEventQueue;

String TOPIC_MEASUREMENTS_AIR_TEMPERATURE =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_AIR_TEMPERATURE);
String TOPIC_MEASUREMENTS_AIR_RELATIVE_HUMIDITY =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_AIR_RELATIVE_HUMIDITY);
String TOPIC_MEASUREMENTS_AIR_DENSITY =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_AIR_DENSITY);
String TOPIC_MEASUREMENTS_AIR_VELOCITY =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_AIR_VELOCITY);
String TOPIC_MEASUREMENTS_DYNAMIC_PRESSURE =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_DYNAMIC_PRESSURE);
String TOPIC_MEASUREMENTS_AIR_FLOW =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_AIR_FLOW);
String TOPIC_MEASUREMENTS_AIR_FLOW_CFM =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_AIR_FLOW_CFM);

String TOPIC_MEASUREMENTS_WIND_TURBINE_RPM =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_WIND_TURBINE_RPM);
String TOPIC_MEASUREMENTS_WIND_TURBINE_POWER =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_WIND_TURBINE_POWER);

String TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT =
    String(MQTT_TOPIC_BASE) +
    String(MQTT_TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT);
String TOPIC_FAN_FREQUENCY_SET_POINT =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_FAN_FREQUENCY_SET_POINT);
String TOPIC_FAN_RPM = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_FAN_RPM);

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
  peripherals::Led *indicatorLed = new peripherals::Led(INDICATOR_LED_PIN);

  services::WifiService *wifi = new services::WifiService(
      WIFI_SSID, WIFI_PASSWORD, WIFI_MAX_RETRY_TIME_MILLISECONDS, indicatorLed);

  WiFiClient *espClient = new WiFiClient();
  PubSubClient *mqttClient = new PubSubClient(*espClient);
  mqttClient->setCallback(mqttSubscriptionCallback);

  services::MqttService *mqtt = new services::MqttService(
      MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD, MQTT_CLIENT_ID,
      MQTT_MAX_RETRY_TIME_MILLISECONDS, mqttClient, indicatorLed);
  services::MqttMessage *mqttMessage;

  while (true) {
    if (!wifi->IsConnected()) {
      wifi->Connect();

    } else if (!mqtt->IsConnected()) {
      if (mqtt->Connect())
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
  peripherals::BermE3fDs30c4 *bermE3fDs30c4 =
      new peripherals::BermE3fDs30c4(BERM_E3F_DS30C4_PIN);
  peripherals::Dht22 *dht22 = new peripherals::Dht22(DHT22_PIN);
  peripherals::Fan *fan = new peripherals::Fan(FAN_CONTROL_PIN);
  peripherals::MPXV7002DP_ADC1115 *mpxv7002dpAdc1115 =
      new peripherals::MPXV7002DP_ADC1115(
          MPXV7002DP_ADC1115_CHANNEL, MPXV7002DP_ADC1115_VOLTAGE_OFFSET,
          MPXV7002DP_ADC1115_VOLTAGE_SENSITIVITY);

  control::Controller *controller = new control::Controller(
      MEASURING_INTERVAL_MILLISECONDS, WIND_TURBINE_PULSE_COUNT_DIVIDER,
      AIR_DENSITY, CROSS_SECTION_AREA, bermE3fDs30c4, dht22, mpxv7002dpAdc1115,
      fan);

  services::MqttMessage *mqttMessage;
  controller->Begin();

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

      float airTemperature = measure.GetAirTemperature();
      if (!isnan(airTemperature)) {
        services::MqttMessage *airTemperatureMessage =
            new services::MqttMessage(TOPIC_MEASUREMENTS_AIR_TEMPERATURE,
                                      airTemperature);
        xQueueSend(MqttPublishingEventQueue, &airTemperatureMessage,
                   pdMS_TO_TICKS(10));
      }

      float airRelativeHumidity = measure.GetAirRelativeHumidity();
      if (!isnan(airRelativeHumidity)) {
        services::MqttMessage *airRelativeHumidityMessage =
            new services::MqttMessage(TOPIC_MEASUREMENTS_AIR_RELATIVE_HUMIDITY,
                                      airRelativeHumidity);
        xQueueSend(MqttPublishingEventQueue, &airRelativeHumidityMessage,
                   pdMS_TO_TICKS(10));
      }

      float airDensity = measure.GetAirDensity();
      if (!isnan(airDensity)) {
        services::MqttMessage *airDensityMessage = new services::MqttMessage(
            TOPIC_MEASUREMENTS_AIR_DENSITY, airDensity);
        xQueueSend(MqttPublishingEventQueue, &airDensityMessage,
                   pdMS_TO_TICKS(10));
      }

      float airVelocity = measure.GetAirVelocity();
      if (!isnan(airVelocity)) {
        services::MqttMessage *airVelocityMessage = new services::MqttMessage(
            TOPIC_MEASUREMENTS_AIR_VELOCITY, airVelocity);
        xQueueSend(MqttPublishingEventQueue, &airVelocityMessage,
                   pdMS_TO_TICKS(10));
      }

      float dynamicPressure = measure.GetDynamicPressure();
      if (!isnan(dynamicPressure)) {
        services::MqttMessage *dynamicPressureMessage =
            new services::MqttMessage(TOPIC_MEASUREMENTS_DYNAMIC_PRESSURE,
                                      dynamicPressure);
        xQueueSend(MqttPublishingEventQueue, &dynamicPressureMessage,
                   pdMS_TO_TICKS(10));
      }

      float airFlow = measure.GetAirFlow();
      if (!isnan(airFlow)) {
        services::MqttMessage *airFlowMessage =
            new services::MqttMessage(TOPIC_MEASUREMENTS_AIR_FLOW, airFlow);
        xQueueSend(MqttPublishingEventQueue, &airFlowMessage,
                   pdMS_TO_TICKS(10));
      }

      float airFlowCfm = measure.GetAirFlowCfm();
      if (!isnan(airFlowCfm)) {
        services::MqttMessage *airFlowCfmMessage = new services::MqttMessage(
            TOPIC_MEASUREMENTS_AIR_FLOW_CFM, airFlowCfm);
        xQueueSend(MqttPublishingEventQueue, &airFlowCfmMessage,
                   pdMS_TO_TICKS(10));
      }

      float windTurbineRpm = measure.GetWindTurbineRpm();
      services::MqttMessage *windTurbineRpmMessage = new services::MqttMessage(
          TOPIC_MEASUREMENTS_WIND_TURBINE_RPM, windTurbineRpm);
      xQueueSend(MqttPublishingEventQueue, &windTurbineRpmMessage,
                 pdMS_TO_TICKS(10));

      float windTurbinePower = measure.GetWindTurbinePower();
      services::MqttMessage *windTurbinePowerMessage =
          new services::MqttMessage(TOPIC_MEASUREMENTS_WIND_TURBINE_POWER,
                                    windTurbinePower);
      xQueueSend(MqttPublishingEventQueue, &windTurbinePowerMessage,
                 pdMS_TO_TICKS(10));

      float fanFrequencySetPoint = measure.GetFanFrequencySetPoint();
      services::MqttMessage *fanFrequencySetPointMessage =
          new services::MqttMessage(TOPIC_FAN_FREQUENCY_SET_POINT,
                                    fanFrequencySetPoint);
      xQueueSend(MqttPublishingEventQueue, &fanFrequencySetPointMessage,
                 pdMS_TO_TICKS(10));

      float fanRpm = measure.GetFanRpm();
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

  MqttPublishingEventQueue = xQueueCreate(11, sizeof(services::MqttMessage *));
  MqttSubscriptionsEventQueue =
      xQueueCreate(1, sizeof(services::MqttMessage *));
}

void loop() {}
