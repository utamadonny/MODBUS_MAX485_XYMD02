/*
  Wiring of Sensor, ESP8266, and MAX485 TTL to RS485 Converter:
  ___________________________________________________________________________________________
  | Sensor (SHT20)   |   MAX485 TTL to RS485 Converter
  |  A (Yellow)      |        A (Terminal block)
  |  B (White)       |        B (Terminal block)
  |  GND (Black)     |       GND (External Supply)
  |  Vs (Red)        |      9-30V (External Supply)
  ___________________________________________________________________________________________
  | MAX485 TTL to RS485 Converter  |       PIN OUT              |         GPIO
  |     RO (Reciever Output)       |        D7 (RX2)            |          13
  |     RE (Reciever Enable)       |        D2                  |          0
  |     DE (Driver Enable)         |        D3                  |          4
  |     DI (Driver Input)          |        D8 (TX2)            |          15
  ___________________________________________________________________________________________
*/
/*
  Wiring of Sensor, ESP32 (Hardware Serial), and MAX485 TTL to RS485 Converter:
  ___________________________________________________________________________________________
  | Sensor (SHT20)   |   MAX485 TTL to RS485 Converter
  |  A (Yellow)      |        A (Terminal block)
  |  B (White)       |        B (Terminal block)
  |  GND (Black)     |       GND (External Supply)
  |  Vs (Red)        |      9-30V (External Supply)
  ___________________________________________________________________________________________
  | MAX485 TTL to RS485 Converter  |       PIN OUT              |         GPIO
  |     RO (Reciever Output)       |        (RX2)               |          13
  |     RE (Reciever Enable)       |        D2                  |          2
  |     DE (Driver Enable)         |        D4                  |          4
  |     DI (Driver Input)          |        (TX2)               |          15
  ___________________________________________________________________________________________
*/
#include <ESP8266WiFi.h>
// #include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include "secrets.h"

#define MAX485_RE_NEG  0
#define MAX485_DE      4
#define SSERIAL_RX_PIN 13
#define SSERIAL_TX_PIN 15
// #define RXD2 16
// #define TXD2 17

//definisi modbus data
#define SLAVE_ID 1
#define READ_ADDRESS 0x001
#define ADDRESS_LENGTH 2

// WiFi Setup
#define WIFI_SSID WIFI_SSIDx
#define WIFI_PASSWORD WIFI_PASSWORDx

// MQTT Setup
#define MQTT_SERVER "test.mosquitto.org"
#define MQTT_PORT 1883

//MQTT Topics
#define MQTT_PUB_TEMP MQTT_PUB_TEMPx
#define MQTT_PUB_HUM  MQTT_PUB_HUMx
#define MQTT_PUB_JSON MQTT_PUB_JSONx

SoftwareSerial Serial2(SSERIAL_RX_PIN, SSERIAL_TX_PIN);
ModbusMaster node;
WiFiClient espClient;
PubSubClient mqtt(espClient);

float temp, hum;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {
  // Initialize control pins
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 9600 baud
  Serial.begin(9600);
  Serial2.begin(9600);

  // Modbus slave ID 1
  node.begin(SLAVE_ID, Serial2);

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("MODBUS IO OK..... connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("WiFi Connected");
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);

  String clientId = String(random(0xffff), HEX);
  mqtt.connect(clientId.c_str());
  Serial.println("MQTT OK.....");
}

void loop() {
  // Request 2 registers starting at 0x0001
   // Data Frame --> 01 04 00 01 00 02 20 0B
  uint8_t result = node.readInputRegisters(READ_ADDRESS, ADDRESS_LENGTH);
  Serial.println("Data Requested");

  if (result == node.ku8MBSuccess) {
    // Get response data from sensor
    temp = node.getResponseBuffer(0) / 10.0f;
    hum = node.getResponseBuffer(1) / 10.0f;
       Serial.print("Temp: "); Serial.print(temp); Serial.print("\t");
       Serial.print("Hum: "); Serial.print(hum);
    Serial.println();
    node.clearResponseBuffer();
    node.clearTransmitBuffer();
    // sendJson(); // kirim data dengan format json
  // sendFloat(); // kirim data dengan format float
  }
  sendJson(); // kirim data dengan format json
  sendFloat(); // kirim data dengan format float
  delay(1000);
}

void sendFloat()
{
  char tmpString[5];
  
  dtostrf(temp, 1, 2, tmpString);
  mqtt.publish(MQTT_PUB_TEMP, tmpString);
  
  dtostrf(hum, 1, 2, tmpString);
  mqtt.publish(MQTT_PUB_HUM, tmpString);
}


void sendJson()
{
  char buffer[256];
  char tmpString[6];
  StaticJsonDocument<256> doc;

  sprintf(tmpString, "%0.1f", temp);
  doc["temperature"] = tmpString;

  sprintf(tmpString, "%0.1f", hum);
  doc["humidity"] = tmpString;

  serializeJson(doc, buffer);
  Serial.println(buffer);
  Serial.println();

  mqtt.publish(MQTT_PUB_JSON, buffer);
}
