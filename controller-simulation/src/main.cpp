#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "parameters.h"
#include "secrets.h"

#include "logger.h"

#include "controller.h"
#include "measurements.h"

#include "berm_e3f_ds30c4.h"
#include "dht22.h"
#include "mpxv7002dp_adc1115.h"

#include "mqtt.h"
#include "wifi_.h"

/**
  TASK HANDLES

  Task Handles are used to manage and reference FreeRTOS tasks. Each task handle
  allows you to:
  - Control task execution (suspend, resume, delete)
  - Monitor task status
  - Assign tasks to specific CPU cores
 */
TaskHandle_t ServerHandlingTask;  // Handle for the server communication task
TaskHandle_t ControlHandlingTask; // Handle for the control and measurement task

/**
  QUEUE HANDLES

  Queue Handles are used to manage FreeRTOS queues. Queues provide thread-safe
  communication between tasks by:
    - Storing messages in a FIFO (First In, First Out) buffer
    - Allowing tasks to send and receive data without conflicts
    - Providing blocking and non-blocking operations
    - Enabling inter-task communication and synchronization
 */
QueueHandle_t
    MqttPublishingEventQueue; // Queue for messages to be published via MQTT
QueueHandle_t MqttSubscriptionsEventQueue; // Queue for incoming MQTT
                                           // subscription messages

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
String TOPIC_FAN_POWER = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_FAN_POWER);

/*
  MQTT SUBSCRIPTION CALLBACK

  This function is called when an MQTT message is
  received. This callback demonstrates queue usage by sending received messages
  to a queue for processing by another task, enabling asynchronous
  messagehandling
*/
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

/**
  SERVER HANDLING TASK

  This is a FreeRTOS task that manages network communications. This task
  demonstrates:
    - Task execution with infinite loop
    - Queue receive operations for processing messages from other tasks
    - Asynchronous communication between tasks via queues

  The task runs continuously, handling WiFi and MQTT connections, and processing
  messages from the publishing queue that are sent by the control task
 */
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

/**
  CONTROL HANDLING TASK

  This is a FreeRTOS task that manages sensor measurements and control. This
  task demonstrates:
    - Task execution with infinite loop
    - Queue operations for both receiving and sending messages
    - Inter-task communication via queues
    - Task delay for timing control

  The task runs continuously, taking sensor measurements and sending them to the
  server task via queues, while also receiving control commands from MQTT
 */
void controlHandling(void *parameter) {
  peripherals::BermE3fDs30c4 *bermE3fDs30c4 =
      new peripherals::BermE3fDs30c4(BERM_E3F_DS30C4_PIN);

  peripherals::Dht22 *dht22 = new peripherals::Dht22(DHT22_PIN);

  peripherals::MPXV7002DP_ADC1115 *mpxv7002dpAdc1115 =
      new peripherals::MPXV7002DP_ADC1115(
          MPXV7002DP_ADC1115_CHANNEL, MPXV7002DP_ADC1115_VOLTAGE_OFFSET,
          MPXV7002DP_ADC1115_VOLTAGE_SENSITIVITY);

  control::Controller *controller = new control::Controller(
      MEASURING_INTERVAL_MILLISECONDS, WIND_TURBINE_PULSE_COUNT_DIVIDER,
      AIR_DENSITY, CROSS_SECTION_AREA,
      CUBIC_METERS_PER_SECOND_TO_CUBIC_FEET_PER_MINUTE, bermE3fDs30c4, dht22,
      mpxv7002dpAdc1115);

  services::MqttMessage *mqttMessage;

  controller->Begin();
  vTaskDelay(pdMS_TO_TICKS(1000));

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

      float fanPower = measure.GetFanPower();
      if (!isnan(fanPower)) {
        services::MqttMessage *fanPowerMessage =
            new services::MqttMessage(TOPIC_FAN_POWER, fanPower);
        xQueueSend(MqttPublishingEventQueue, &fanPowerMessage,
                   pdMS_TO_TICKS(10));
      }
    }
  }
}

/**
  SETUP FUNCTION

  Initializes the ESP32 system with tasks and queues. This function demonstrates
  the fundamental concepts of ESP32 multitasking:

  TASK CREATION:
    - xTaskCreatePinnedToCore creates FreeRTOS tasks and assigns them to
  specific CPU cores.
    - Each task has its own stack space (in words, max 8191), priority, and can run independently.
    - Tasks communicate through queues, enabling concurrent execution.

  QUEUE CREATION:
    - xQueueCreate allocates memory for message queues.
    - Queues provide thread-safe communication between tasks.
    - Different queue sizes accommodate different message volumes.
 */
void setup() {
  xTaskCreatePinnedToCore(serverHandling, "serverHandling", 10000, NULL, 1,
                          &ServerHandlingTask, 0);
  xTaskCreatePinnedToCore(controlHandling, "controlHandling", 10000, NULL, 1,
                          &ControlHandlingTask, 1);

  MqttPublishingEventQueue = xQueueCreate(12, sizeof(services::MqttMessage *));
  MqttSubscriptionsEventQueue =
      xQueueCreate(1, sizeof(services::MqttMessage *));
}

void loop() {}
