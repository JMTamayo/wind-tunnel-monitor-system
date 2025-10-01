#define WIFI_MAX_RETRY_TIME_MILLISECONDS 5000 // ms
#define MQTT_MAX_RETRY_TIME_MILLISECONDS 5000 // ms

// Configuration parameters for MQTT:
#define MQTT_TOPIC_BASE                                                        \
  "/Thingworx/ThingWorx.WindTunnelControlSystem.Controller"
#define MQTT_TOPIC_AIR_TEMPERATURE "/airTemperature"
#define MQTT_TOPIC_AIR_RELATIVE_HUMIDITY "/airRelativeHumidity"
#define MQTT_TOPIC_AIR_DENSITY "/airDensity"
#define MQTT_TOPIC_AIR_VELOCITY "/airVelocity"
#define MQTT_TOPIC_DYNAMIC_PRESSURE "/airDynamicPressure"
#define MQTT_TOPIC_AIR_FLOW "/airFlow"
#define MQTT_TOPIC_AIR_FLOW_CFM "/airFlowCFM"
#define MQTT_TOPIC_WIND_TURBINE_RPM "/turbineRpm"
#define MQTT_TOPIC_WIND_TURBINE_POWER "/turbinePower"
#define MQTT_TOPIC_CONTROL_FAN_FREQUENCY_SET_POINT                             \
  "/fanFrequencySetPointRequest"
#define MQTT_TOPIC_FAN_FREQUENCY_SET_POINT "/fanFrequencySetPoint"
#define MQTT_TOPIC_FAN_RPM "/fanRpm"
#define MQTT_TOPIC_FAN_TOTAL_ACTIVE_POWER "/fanPower"

#define MEASURING_INTERVAL_MILLISECONDS 2000 // ms

// Configuration parameters for wind tunnel:
#define CROSS_SECTION_AREA 1.7 // m^2

// Configuration parameters for BERM-E3F-DS30C4:
#define BERM_E3F_DS30C4_PIN 34
#define WIND_TURBINE_PULSE_COUNT_DIVIDER 3 // pulses/revolution

// Configuration parameters for DHT22:
#define DHT22_PIN 32

// Configuration parameters for FAN:
#define FAN_CONTROL_PIN 23

// Configuration parameters for indicator LED for network connections:
#define INDICATOR_LED_PIN 2

// Configuration parameters for MPXV7002DP-ADC1115:
#define MPXV7002DP_ADC1115_CHANNEL 0
#define MPXV7002DP_ADC1115_VOLTAGE_OFFSET 2.5      // V
#define MPXV7002DP_ADC1115_VOLTAGE_SENSITIVITY 1.0 // V/kPa
#define AIR_DENSITY 1.10                           // kg/m^3

// Configuration parameters for SDM630MCT:
#define SDM630MCT_RX2_PIN 16
#define SDM630MCT_TX2_PIN 17
#define SDM630MCT_DEVICE_ADDRESS 1
#define SDM630MCT_TOTAL_ACTIVE_POWER_ADDRESS 52