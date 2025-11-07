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
    El presente dispositivo tiene como objetivo monitorizar las variables de operación de un
    prototipo de turbina eólica para evaluar la eficiencia de la generación de energía en el
    equipo. Los datos recopilados son enviados a un servidor MQTT y aprovechados en la 
    plataforma de PTC ThingWorx para efectos de análisis y control.

    Para su uso, es necesario crear un módulo "secrets.h" en el cual se incluyan los secretos
    para la gestión de la conexión con el servicio WiFi y el servidor MQTT. Las variables que
    se deben instanciar en él son:
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
#include <WiFi.h>
#include <PubSubClient.h>

// Importación de Parámetros y Secretos:
#include "secrets.h"
#include "parameters.h"

// Definición de Variables:
WiFiClient *espClient = new WiFiClient();
PubSubClient *mqttClient = new PubSubClient(*espClient);

String TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT =
  String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT);
String TOPIC_MOTOR_RPM = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_MOTOR_RPM);

String TOPIC_TURBINE_RPM = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_TURBINE_RPM);
String TOPIC_TURBINE_CURRENT_AMP = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_TURBINE_CURRENT_AMP);
String TOPIC_TURBINE_VOLTAGE_V = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_TURBINE_VOLTAGE_V);
String TOPIC_TURBINE_POWER_W = String(MQTT_TOPIC_BASE) + String(MQTT_TOPIC_TURBINE_POWER_W);

float motorFrequencySetPoint = 0;
float motorRPM = 0;

float turbineRPM = 0;
float turbineCurrentAMP = 0;
float turbineVoltageV = 0;
float turbinePowerW = 0;

float lastSamplingTime = 0;

// Definición de Subrutinas y Funciones:

/*
  Indicar Problemas de Red

  Descripción:
    Permite encender o apagar el LED incorporado en la placa como recurso
    para identificar visualmente si hay problemas de red.
*/
void networkLED(bool on) {
  unsigned int level = on ? HIGH : LOW;
  digitalWrite(BUILTIN_LED_PIN, level);
}

/*
  Conectarse a Servicio WiFi

  Descripción:
    Conectarse a internet vía WiFi a partir de usuario y contraseña de
    la red. Cuando la comunicación se establece exitosamente, el led
    incorporado de la placa estará encendido.
*/
void connectToWiFi() {
  networkLED(false);

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

  networkLED(true);
}

/*
  Publicar Eventos en Servidor MQTT

  Descripción:
    Función que permite publicar en servidor MQTT los valores de las variables
    de operación del sistema
*/
void publishToMQTT() {
  Serial.println("[MQTT] Starting to publish variables to MQTT server");

  bool ok;

  ok = mqttClient->publish(TOPIC_MOTOR_RPM.c_str(), String(motorRPM).c_str());
  if (ok) {
    Serial.print("[MQTT] Topic: " + TOPIC_MOTOR_RPM);
    Serial.println(". Payload: " + String(motorRPM));
  }

  ok = mqttClient->publish(TOPIC_TURBINE_RPM.c_str(), String(turbineRPM).c_str());
  if (ok) {
    Serial.print("[MQTT] Topic: " + TOPIC_TURBINE_RPM);
    Serial.println(". Payload: " + String(turbineRPM));
  }

  ok = mqttClient->publish(TOPIC_TURBINE_CURRENT_AMP.c_str(), String(turbineCurrentAMP).c_str());
  if (ok) {
    Serial.print("[MQTT] Topic: " + TOPIC_TURBINE_CURRENT_AMP);
    Serial.println(". Payload: " + String(turbineCurrentAMP));
  }

  ok = mqttClient->publish(TOPIC_TURBINE_VOLTAGE_V.c_str(), String(turbineVoltageV).c_str());
  if (ok) {
    Serial.print("[MQTT] Topic: " + TOPIC_TURBINE_VOLTAGE_V);
    Serial.println(". Payload: " + String(turbineVoltageV));
  }

  ok = mqttClient->publish(TOPIC_TURBINE_POWER_W.c_str(), String(turbinePowerW).c_str());
  if (ok) {
    Serial.print("[MQTT] Topic: " + TOPIC_TURBINE_POWER_W);
    Serial.println(". Payload: " + String(turbinePowerW));
  }

  Serial.println("[MQTT] Variables succesfully published");
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

  Serial.print("[MQTT] A message was received from the MQTT server");
  Serial.println(". Topic: " + String(topicStr) + ". Payload: " + String(payloadStr));

  if (topicStr == TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT) {
    motorFrequencySetPoint = constrain(payloadStr.toFloat(), 0.0f, 100.0f);
    modifyMotorFrequencySetPoint();

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
  networkLED(false);

  unsigned long startTimeMs = millis();
  unsigned long retryTimeMs = 0;

  while (!mqttClient->connected()) {
    if (mqttClient->connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
      break;

    if (retryTimeMs >= MQTT_MAX_RETRY_TIME_MS) {
      Serial.println("[MQTT] Connection to MQTT server failed. Server: " + String(MQTT_SERVER));
      return;
    }

    delay(DEFAULT_CONNECTION_DELAY_TIME_MS);
    retryTimeMs = millis() - startTimeMs;
  }

  Serial.print("[MQTT] Connected to MQTT server. Server: " + String(MQTT_SERVER));
  Serial.println(". Client ID: " + String(MQTT_CLIENT_ID));

  mqttClient->subscribe(TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT.c_str());
  Serial.println("[MQTT] Subscribed to the topic: " + TOPIC_CONTROL_MOTOR_FREQUENCY_SET_POINT);

  networkLED(true);
}

/*
  Modificar Frecuencia de Giro del Motor

  
*/
void modifyMotorFrequencySetPoint() {
  Serial.print("[CONTROL] Starting to modify the motor frequency set point to ");
  Serial.println(String(motorFrequencySetPoint));

  /*
    TODO: Implementar la lógica para cambiar la frecuencia del motor.
  */

  Serial.println("[CONTROL] Motor frequency changed to " + String(motorFrequencySetPoint));
}

void readMotorValues() {
  Serial.println("[MEASURE] Starting to read the motor properties.");

  /*
    TODO: Implementar la lógica para leer las propiedades del motor.

    Los siguientes valores son para efectos depruebas.
  */
  motorRPM = random(700, 710);
  /* --- */

  Serial.println("[MEASURE] Turbine properties successfully read.");
}

void readTurbineValues() {
  Serial.println("[MEASURE] Starting to read the turbine properties.");

  /*
    TODO: Implementar la lógica para leer las propiedades de la turbina.

    Los siguientes valores son para efectos depruebas.
  */
  turbineRPM = motorRPM;
  turbineCurrentAMP = random(1500, 1600) / 1000.0f;
  turbineVoltageV = random(108, 112);
  turbinePowerW = turbineCurrentAMP * turbineVoltageV;
  /* --- */

  Serial.println("[MEASURE] Turbine properties successfully read.");
}

// Subrutina para Configuración del Proceso a Ejecutar:
void setup() {
  // Configuración de pines I/O:
  pinMode(BUILTIN_LED_PIN, OUTPUT);

  // Limpieza de salidas:
  networkLED(false);
  modifyMotorFrequencySetPoint();

  // Configuración de comunicaciones:
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;

  connectToWiFi();

  mqttClient->setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient->setCallback(mqttSubscriptionCallback);
  connectToMQTT();

  // Inicio de variables de proceso:
  lastSamplingTime = millis();
}

// Loop principal, en el cual se gestionan las conexiones, las acciones de control y monitoreo:
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
