/*
  Prototipo de Monitoreo de Variables de Generación en Turbina Eólica

  Autores:
    - Mateo Zapata Quintero
    - Samuel Arango Palacio
    - Camilo Rodriguez Londoño
    - Juan Manuel Tamayo Gutiérrez

  Version:
    1.0.0

  Descripción:
    El presente dispositivo tiene como objetivo monitorizar las variables de
  operación de un prototipo de turbina eólica para evaluar la eficiencia de la
  generación de energía en el equipo. Los datos recopilados son enviados a un
  servidor MQTT y aprovechados en la plataforma de PTC ThingWorx para efectos de
  análisis y control.

    Para su uso, es necesario crear un módulo "secrets.h" en el cual se incluyan
  los secretos para la gestión de la conexión con el servicio WiFi y el servidor
  MQTT. Las variables que se deben instanciar en él son:
      - WIFI_SSID
      - WIFI_PASSWORD
      - MQTT_SERVER
      - MQTT_PORT
      - MQTT_USER
      - MQTT_PASSWORD
      - MQTT_CLIENT_ID
      - MQTT_TOPIC_BASE

    Universidad EAFIT
    Medellín, Colombia
    2025
*/

// Importación de Librerías:
#include "driver/ledc.h"
#include <Adafruit_ADS1X15.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

// Importación de Parámetros y Secretos:
#include "parameters.h"
#include "secrets.h"

// Definición de Variables:
WiFiClient *espClient = new WiFiClient();
PubSubClient *mqttClient = new PubSubClient(*espClient);

String TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT =
    String(MQTT_TOPIC_BASE) +
    String(MQTT_TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT);
String TOPIC_CONTROL_RELAY_NUMBER_REQUEST =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_CONTROL_RELAY_NUMBER_REQUEST);

String TOPIC_MOTOR_FREQUENCY_SET_POINT =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_MOTOR_FREQUENCY_SET_POINT);
String TOPIC_RELAY_NUMBER =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_RELAY_NUMBER);
String TOPIC_MOTOR_RPM = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_MOTOR_RPM);
String TOPIC_MOTOR_CURRENT_AMP =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_MOTOR_CURRENT_AMP);
String TOPIC_MOTOR_VOLTAGE_V =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_MOTOR_VOLTAGE_V);
String TOPIC_MOTOR_POWER_W =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_MOTOR_POWER_W);

String TOPIC_TURBINE_RPM =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_TURBINE_RPM);
String TOPIC_TURBINE_CURRENT_AMP =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_TURBINE_CURRENT_AMP);
String TOPIC_TURBINE_VOLTAGE_V =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_TURBINE_VOLTAGE_V);
String TOPIC_TURBINE_POWER_W =
    String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_TURBINE_POWER_W);

float motorFrequencySetPointRequest = 0;
unsigned int relayNumber = 3;

float motorFrequencySetPoint = 0;
float motorRPM = 0;
float motorCurrentAMP = 0;
float motorVoltageV = 0;
float motorPowerW = 0;

float turbineRPM = 0;
float turbineCurrentAMP = 0;
float turbineVoltageV = 0;
float turbinePowerW = 0;

volatile unsigned long speedEncoderPulseCount = 0;
unsigned long lastSamplingTime = 0;

Adafruit_ADS1115 ads;

ledc_timer_config_t timerConfig;
ledc_channel_config_t channelConfig;

// Definición de Subrutinas y Funciones:

/*
  Administra la lectura del encoder de velocidad

  Descripción:
    Función acoplada al pin de lectura digital desde el encoder, la cual
    incrementa el conteo de pulsos cada vez que se registra una lectura en
    alto.
*/
void IRAM_ATTR handleSpeedEncoder() { speedEncoderPulseCount++; }

/*
  Conectarse a Servicio WiFi

  Descripción:
    Conectarse a internet vía WiFi a partir de usuario y contraseña de
    la red. Cuando la comunicación se establece exitosamente, el led
    incorporado de la placa estará encendido.
*/
void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);

  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (WiFi.status() != WL_CONNECTED) {
    if (retryTimeMs >= WIFI_MAX_RETRY_TIME_MS) {
      Serial.print("[WIFI] Connection to WiFi network failed. SSID: ");
      Serial.println(String(WIFI_SSID));
      return;
    }

    delay(DEFAULT_CONNECTION_DELAY_TIME_MS);
    retryTimeMs = millis() - startTimeMs;
  }

  Serial.print("[WIFI] Connected to WiFi network. SSID: " + String(WIFI_SSID));
  Serial.println(String(". IP: ") + String(WiFi.localIP().toString().c_str()));
}

/*
  Publicar Eventos en Servidor MQTT

  Descripción:
    Función que permite publicar en servidor MQTT los valores de las variables
    de operación del sistema
*/
void publishToMQTT() {
  bool ok;

  ok = mqttClient->publish(TOPIC_MOTOR_FREQUENCY_SET_POINT.c_str(),
                           String(motorFrequencySetPoint).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " +
                 TOPIC_MOTOR_FREQUENCY_SET_POINT);
    Serial.println(". Payload: " + String(motorFrequencySetPoint));
  }

  ok = mqttClient->publish(TOPIC_RELAY_NUMBER.c_str(),
                           String(relayNumber).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_RELAY_NUMBER);
    Serial.println(". Payload: " + String(relayNumber));
  }

  ok = mqttClient->publish(TOPIC_MOTOR_RPM.c_str(), String(motorRPM).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_MOTOR_RPM);
    Serial.println(". Payload: " + String(motorRPM));
  }

  ok = mqttClient->publish(TOPIC_MOTOR_CURRENT_AMP.c_str(),
                           String(motorCurrentAMP).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_MOTOR_CURRENT_AMP);
    Serial.println(". Payload: " + String(motorCurrentAMP));
  }

  ok = mqttClient->publish(TOPIC_MOTOR_VOLTAGE_V.c_str(),
                           String(motorVoltageV).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_MOTOR_VOLTAGE_V);
    Serial.println(". Payload: " + String(motorVoltageV));
  }

  ok = mqttClient->publish(TOPIC_MOTOR_POWER_W.c_str(),
                           String(motorPowerW).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_MOTOR_POWER_W);
    Serial.println(". Payload: " + String(motorPowerW));
  }

  ok = mqttClient->publish(TOPIC_TURBINE_RPM.c_str(),
                           String(turbineRPM).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_TURBINE_RPM);
    Serial.println(". Payload: " + String(turbineRPM));
  }

  ok = mqttClient->publish(TOPIC_TURBINE_CURRENT_AMP.c_str(),
                           String(turbineCurrentAMP).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_TURBINE_CURRENT_AMP);
    Serial.println(". Payload: " + String(turbineCurrentAMP));
  }

  ok = mqttClient->publish(TOPIC_TURBINE_VOLTAGE_V.c_str(),
                           String(turbineVoltageV).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_TURBINE_VOLTAGE_V);
    Serial.println(". Payload: " + String(turbineVoltageV));
  }

  ok = mqttClient->publish(TOPIC_TURBINE_POWER_W.c_str(),
                           String(turbinePowerW).c_str());
  if (ok) {
    Serial.print("[MQTT] Data published. Topic: " + TOPIC_TURBINE_POWER_W);
    Serial.println(". Payload: " + String(turbinePowerW));
  }
}

/*
  Callback para Eventos Suscritos en Servidor MQTT

  Descripción:
    Función que se ejecuta cuando se recibe un mensaje de un tópico suscrito
    en servidor MQTT.
*/
void mqttSubscriptionCallback(char *topic, byte *payload, unsigned int length) {
  String topicStr = topic;
  String payloadStr = (char *)payload;
  payloadStr.remove(length);

  Serial.println("[MQTT] Data received. Topic: " + topicStr +
                 ". Payload: " + payloadStr);

  if (topicStr.equals(TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT)) {
    motorFrequencySetPointRequest =
        constrain(payloadStr.toFloat(), 0.0f, 100.0f);
    modifyMotorFrequencySetPointRequest();

  } else if (topicStr.equals(TOPIC_CONTROL_RELAY_NUMBER_REQUEST)) {
    selectRelay(payloadStr.toInt());

  } else {
    Serial.println("[MQTT] Recived message is not supported");
  }
}

/*
  Conectarse a Servidor MQTT

  Descripción:
    Conectarse a servidor MQTT a partir de su url, puerto, usuario y contraseña.
    Adicionalmente, se suscribe al tópico para el control de la frecuencia de
    giro del motor. Cuando la comunicación se establece exitosamente, el led
    incorporado de la placa estará encendido.
*/
void connectToMQTT() {
  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (!mqttClient->connected()) {
    if (mqttClient->connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
      break;

    if (retryTimeMs >= MQTT_MAX_RETRY_TIME_MS) {
      Serial.println("[MQTT] Connection to MQTT server failed. Server: " +
                     String(MQTT_SERVER));
      return;
    }

    delay(DEFAULT_CONNECTION_DELAY_TIME_MS);
    retryTimeMs = millis() - startTimeMs;
  }

  Serial.print("[MQTT] Connected to MQTT server. Server: " +
               String(MQTT_SERVER));
  Serial.println(". Client ID: " + String(MQTT_CLIENT_ID));

  mqttClient->subscribe(TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT.c_str());
  Serial.println("[MQTT] Subscribed to the topic: " +
                 TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT);

  mqttClient->subscribe(TOPIC_CONTROL_RELAY_NUMBER_REQUEST.c_str());
  Serial.println("[MQTT] Subscribed to the topic: " +
                 TOPIC_CONTROL_RELAY_NUMBER_REQUEST);
}

/*
  Modificar Frecuencia de Giro del Motor

  Descripción:
    Función que permite mofificar la frecuencia de giro del motor a partir del
  dato definido por el usuario.
*/
void modifyMotorFrequencySetPointRequest() {
  int duty = map(motorFrequencySetPointRequest, 0, 100, 0, 1023);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)PWM_CHANNEL);
  motorFrequencySetPoint = motorFrequencySetPointRequest;

  Serial.println("[CONTROL] Motor frequency changed to " +
                 String(motorFrequencySetPointRequest));
}

void selectRelay(int num) {
  switch (num) {
  case 1:
    digitalWrite(RELAY_TWO_PIN, LOW);
    digitalWrite(RELAY_THREE_PIN, LOW);
    digitalWrite(RELAY_ONE_PIN, HIGH);
    break;

  case 2:
    digitalWrite(RELAY_ONE_PIN, LOW);
    digitalWrite(RELAY_THREE_PIN, LOW);
    digitalWrite(RELAY_TWO_PIN, HIGH);
    break;

  case 3:
    digitalWrite(RELAY_ONE_PIN, LOW);
    digitalWrite(RELAY_TWO_PIN, LOW);
    digitalWrite(RELAY_THREE_PIN, HIGH);
    break;

  default:
    Serial.println("[CONTROL] Relay number unknown. No action taken.");
    return;
  }

  relayNumber = num;

  Serial.println("[CONTROL] Relay changed to #" + String(num));
}

void readMotorValues() {
  // Leer RPM del motor:
  detachInterrupt(digitalPinToInterrupt(SPEED_ENCODER_PIN));
  unsigned long currentTime = millis();
  unsigned long pulses = speedEncoderPulseCount;
  speedEncoderPulseCount = 0;
  attachInterrupt(digitalPinToInterrupt(SPEED_ENCODER_PIN), handleSpeedEncoder,
                  RISING);
  float encoderRPM = (float)MINUTE_TO_MILLISECONDS_FACTOR * (float)pulses /
                     ((float)currentTime - (float)lastSamplingTime);
  motorRPM = encoderRPM / (float)ENCODER_REDUCTION_FACTOR;

  // Leer voltaje del motor:
  int16_t A2raw = ads.readADC_SingleEnded(2);
  float aux2 = ads.computeVolts(A2raw) *
               (MOTOR_VOLTAGE_DIVIDER_R2 + MOTOR_VOLTAGE_DIVIDER_R1) /
               (MOTOR_VOLTAGE_DIVIDER_R2);
  motorVoltageV = 18.0 - aux2;

  // Leer corriente del motor:
  int16_t A1raw = ads.readADC_SingleEnded(1);
  motorCurrentAMP = max(0.0f, ads.computeVolts(A1raw) -
                                  (float)ZERO_VOLTS_VALUE_MOTOR_CURRENT) /
                    (float)WSC1800_SENSIBILITY;

  // Calcular potencia del motor:
  motorPowerW = motorCurrentAMP * motorVoltageV;
}

void readTurbineValues() {
  turbineRPM = motorRPM;

  float resistance;
  switch (relayNumber) {
  case 1:
    resistance = RESISTANCE_ONE_VALUE;
  case 2:
    resistance = RESISTANCE_TWO_VALUE;
  case 3:
    resistance = RESISTANCE_THREE_VALUE;
  }

  int16_t A0raw = ads.readADC_SingleEnded(0);
  turbineCurrentAMP = max(0.0f, ads.computeVolts(A0raw) -
                                    (float)ZERO_VOLTS_VALUE_TURBINE_CURRENT) /
                      (float)WSC1800_SENSIBILITY;

  turbineVoltageV = turbineCurrentAMP * resistance;
  turbinePowerW = turbineVoltageV * turbineCurrentAMP;

  // Calcular potencia de la turbina:
  turbinePowerW = turbineCurrentAMP * turbineVoltageV;
}

// Subrutina para Configuración del Proceso a Ejecutar:
void setup() {
  // Configuración de Serial para debuggin:
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;
  Serial.println("[SETUP] Serial successfully started.");

  Wire.begin(SDA_PIN, SCL_PIN);
  ads.begin(0x48);

  // Configuración de pines I/O:
  pinMode(SPEED_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SPEED_ENCODER_PIN), handleSpeedEncoder,
                  RISING);

  pinMode(MOTOR_FREQUENCY_CONTROL_PIN, OUTPUT);
  pinMode(RELAY_ONE_PIN, OUTPUT);
  pinMode(RELAY_TWO_PIN, OUTPUT);
  pinMode(RELAY_THREE_PIN, OUTPUT);

  // Limpieza de salidas:
  analogWrite(MOTOR_FREQUENCY_CONTROL_PIN, 0);
  digitalWrite(RELAY_ONE_PIN, LOW);
  digitalWrite(RELAY_TWO_PIN, LOW);
  digitalWrite(RELAY_THREE_PIN, HIGH);

  // Configuración del ADS:
  ads.setGain(GAIN_ONE);

  // Configurar el temporizador PWM:
  timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
  timerConfig.timer_num = LEDC_TIMER_0;
  timerConfig.duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION;
  timerConfig.freq_hz = PWM_FREQUENCY;
  timerConfig.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timerConfig);

  // Configurar el canal PWM
  channelConfig.gpio_num = MOTOR_FREQUENCY_CONTROL_PIN;
  channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
  channelConfig.channel = (ledc_channel_t)PWM_CHANNEL;
  channelConfig.intr_type = LEDC_INTR_DISABLE;
  channelConfig.timer_sel = LEDC_TIMER_0;
  channelConfig.duty = 0;
  channelConfig.hpoint = 0;
  ledc_channel_config(&channelConfig);

  // Configuración de comunicaciones:
  connectToWiFi();

  mqttClient->setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient->setCallback(mqttSubscriptionCallback);
  connectToMQTT();

  // Inicio de variables de proceso:
  // lastSamplingTime = millis();
}

// Loop principal, en el cual se gestionan las conexiones, las acciones de
// control y monitoreo:
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();

  } else if (!mqttClient->connected()) {
    connectToMQTT();

  } else {
    mqttClient->loop();

    if (millis() - lastSamplingTime >= SAMPLING_TIME_MS) {
      readMotorValues();
      readTurbineValues();

      publishToMQTT();

      lastSamplingTime = millis();
    }
  }
}
