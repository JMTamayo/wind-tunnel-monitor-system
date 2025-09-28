#define WIFI_MAX_RETRY_TIME_MILLISECONDS 5000

#define MQTT_MAX_RETRY_TIME_MILLISECONDS 5000

#define MQTT_TOPIC_BASE                                                        \
  "/Thingworx/ThingWorx.WindTunnelControlSystem.Controller"

#define MQTT_TOPIC_AIR_TEMPERATURE "/airTemperature"
#define MQTT_TOPIC_AIR_RELATIVE_HUMIDITY "/airRelativeHumidity"

#define MQTT_TOPIC_WIND_TURBINE_RPM "/turbineRpm"

#define MQTT_TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT                             \
  "/fanFrequencySetPointRequest"
#define MQTT_TOPIC_FAN_FREQUENCY_SET_POINT "/fanFrequencySetPoint"
#define MQTT_TOPIC_FAN_RPM "/fanRpm"

#define MEASURING_INTERVAL_MILLISECONDS 2000

#define FAN_CONTROL_PIN 23

#define BERM_E3F_DS30C4_PIN 22
#define WIND_TURBINE_PULSE_COUNT_DIVIDER 3

#define DHT22_PIN 34
