#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <time.h>

#include "config.h"

#include "collector.h"
#include "led.h"
#include "logger.h"
#include "mqtt.h"
#include "time_.h"
#include "wifi_.h"

WiFiClient *espClient;
PubSubClient *mqttClient;

peripherals::Led *builtInLed;

sampling::Collector *collector;

services::MqttService *mqtt;
services::TimeService *timeKeeper;
services::WifiService *wifi;

void setup() {
  logging::logger->Initialize();
  logging::logger->Info("Setting up system");

  builtInLed = new peripherals::Led(BUILTIN_LED_PIN);

  wifi = new services::WifiService(
      WIFI_SSID, WIFI_PASSWORD, WIFI_MAX_RETRY_TIME_MILLISECONDS, builtInLed);

  timeKeeper = new services::TimeService(
      TIMING_NTP_SERVER, TIMING_GMT_OFFSET_SECONDS,
      TIMING_DAYLIGHT_OFFSET_SECONDS, TIMING_MAX_RETRY_TIME_MILLISECONDS);

  espClient = new WiFiClient();
  mqttClient = new PubSubClient(*espClient);
  mqtt = new services::MqttService(
      MQTT_SERVER, MQTT_USER, MQTT_PASSWORD, MQTT_CLIENT_ID,
      MQTT_MAX_RETRY_TIME_MILLISECONDS, MQTT_TOPIC_BASE, MQTT_TOPIC_AIR_FLOW,
      MQTT_TOPIC_AIR_FLOW_CFM, MQTT_TOPIC_AIR_PRESSURE,
      MQTT_TOPIC_AIR_TEMPERATURE, MQTT_TOPIC_AIR_VELOCITY,
      MQTT_TOPIC_FAN_FREQUENCY, MQTT_TOPIC_FAN_FREQUENCY_SETPOINT,
      MQTT_TOPIC_FAN_STATE, MQTT_TOPIC_TIMESTAMP,
      MQTT_TOPIC_TURBINE_AXIAL_FORCE, MQTT_TOPIC_TURBINE_TRANSVERSE_FORCE,
      MQTT_TOPIC_TURBINE_POWER, builtInLed, mqttClient);

  wifi->Connect();
  mqtt->Connect();

  collector =
      new sampling::Collector(SAMPLING_INTERVAL_MILLISECONDS, timeKeeper);

  logging::logger->Info("System setup complete");
}

void loop() {
  if (!wifi->IsConnected()) {
    wifi->Connect();
  }

  if (!mqtt->IsConnected()) {
    mqtt->Connect();
  }

  mqtt->Loop();

  if (collector->IsSamplingTimeReached()) {
    sampling::Sample sample = collector->Collect();
    mqtt->Publish(sample);
  }
}
