#include <vector>
#include <utility>
#include <math.h>

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsClient.h>
#include "time.h"
#include <Wire.h>
#include "Bitcoin.h"
#include "Hash.h"
#include <esp_random.h>
#include "QRCode.h"
#include <SPIFFS.h>
#include <ESP32Ping.h>
#include "wManager.h"
#include <ArduinoJson.h>
#include "config.h"

#include <NostrEvent.h>
#include "nostr.h"
#include "nip19.h"

#include "sensor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP280_I2C_ADDRESS 0x76

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C

#define PARAM_FILE "/elements.json"

int triggerAp = false;

bool lastInternetConnectionState = true;

int socketDisconnectedCount = 0;

WebSocketsClient webSocket;

String serialisedEventRequest;

extern bool hasInternetConnection;

char const *deviceIdentifier = "ctdevice1";

fs::SPIFFSFS &FlashFS = SPIFFS;
#define FORMAT_ON_FAIL true

// define funcs
void configureAccessPoint();
void initWiFi();
bool whileCP(void);
unsigned long getUnixTimestamp();
// void noteEvent(const std::string& key, const char* payload);
void okEvent(const std::string& key, const char* payload);
void nip01Event(const std::string& key, const char* payload);
void relayConnectedEvent(const std::string& key, const std::string& message);
void relayDisonnectedEvent(const std::string& key, const std::string& message);
uint16_t getRandomNum(uint16_t min, uint16_t max);
void loadSettings();
void connectToNostrRelays();

// Define the WiFi event callback function
void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("Connected to WiFi and got an IP");
      connectToNostrRelays();      
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi");
      // WiFi.begin(ssid, password); // Try to reconnect after getting disconnected
      break;
  }
}

void broadcastSensorData() {
    // get the sensor data, create the event and push it to the queue
  unsigned long timestamp = getUnixTimestamp();
  float temperature = bmp.readTemperature();
  Serial.println("Temperature: " + String(temperature) + "°C");

  StaticJsonDocument<512> tagsDoc;
  JsonArray tags = tagsDoc.to<JsonArray>();

  // Add device tag
  JsonArray typeTag = tags.createNestedArray();
  typeTag.add("l");
  typeTag.add("type");
  typeTag.add("temperature");

  // // Add alt tag
  JsonArray altTag = tags.createNestedArray();
  altTag.add("unit");
  altTag.add("celcius");

  JsonArray dTag = tags.createNestedArray();
  dTag.add("d");
  dTag.add(String(deviceIdentifier) + "-temperature");

  String tempString = String(temperature);
  String tagsString;
  serializeJson(tags, tagsString);
  
  String noteString = nostr::getNote(nsecHex, npubHex, timestamp, tempString, 30107, tagsString);
  Serial.println("Note string: " + noteString);
  noteString = "[\"EVENT\", " + noteString + "]";
  webSocket.sendTXT(noteString);


  float pressure = bmp.readPressure() / 100.0F;
  Serial.println("pressure: " + String(pressure) + "hPa");

  StaticJsonDocument<512> pressureTagsDoc;
  JsonArray pressureTags = pressureTagsDoc.to<JsonArray>();

  JsonArray pressureTypeTag = pressureTags.createNestedArray();
  pressureTypeTag.add("l");
  pressureTypeTag.add("type");
  pressureTypeTag.add("pressure");
  JsonArray pressureUnitTag = pressureTags.createNestedArray();
  pressureUnitTag.add("unit");
  pressureUnitTag.add("hPa");
  JsonArray pressureDTag = pressureTags.createNestedArray();
  pressureDTag.add("d");
  pressureDTag.add(String(deviceIdentifier) + "-pressure");
  String pressureTagsString;
  serializeJson(pressureTags, pressureTagsString);

  String pressureString = String(pressure);
  timestamp = getUnixTimestamp();
  noteString = nostr::getNote(nsecHex, npubHex, timestamp, pressureString, 30107, pressureTagsString);
  noteString = "[\"EVENT\", " + noteString + "]";
  delay(1000);
  webSocket.sendTXT(noteString);
}

uint8_t socketDisconnectCount = 0;

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    ++socketDisconnectCount;
    if(socketDisconnectCount > 3) {
      Serial.println("Error. Failed to connect to relay.");
      ESP.restart();
    }
    Serial.printf("[WSc] Disconnected!\n");
    Serial.println("Disconnected from relay. Reconnecting..");
    webSocket.beginSSL(relayUrl, 443);
    webSocket.onEvent(webSocketEvent);
    break;
  case WStype_CONNECTED:
  {
    socketDisconnectCount = 0;
    Serial.printf("[WSc] Connected to %s\n", relayUrl);
    Serial.println("Connected to relay Awaiting requests.");
    broadcastSensorData();
    break;
  }
  case WStype_TEXT:
  {
    Serial.printf("[WSc] Got WS TEXT\n");
    Serial.println("Payload is: " + String((char *)payload));
    // handleWebSocketMessage(NULL, payload, length);
    break;
  }
  case WStype_BIN:
    Serial.printf("[WSc] get binary length: %u\n", length);
    break;
  }
}

/**
 * @brief Connect to the Nostr relays
 * 
 */
void connectToNostrRelays() {
  webSocket.beginSSL(relayUrl, 443);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

// Function that gets current epoch time
unsigned long getUnixTimestamp() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

String lastPayload = "";


uint16_t getRandomNum(uint16_t min, uint16_t max) {
  uint16_t rand  = (esp_random() % (max - min + 1)) + min;
  Serial.println("Random number: " + String(rand));
  return rand;
}

long bootTime = 0;


void setup() {
  Serial.begin(115200);
  Serial.println("boot");

  bootTime = millis();

  unsigned status;
  status = bmp.begin(BMP280_I2C_ADDRESS);
    if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  FlashFS.begin(FORMAT_ON_FAIL);
  // init spiffs
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  randomSeed(analogRead(0)); // Seed the random number generator

  WiFi.onEvent(WiFiEvent);
  init_WifiManager();

  if(hasInternetConnection) {
    Serial.println("Has internet connection. Connectring to relays");
    connectToNostrRelays();
  }

  const char* ntpServer = "uk.pool.ntp.org";
  long gmtOffset_sec = 0;
  long daylightOffset_sec = 0;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

}

long lastInternetConnectionCheckTime = 0;
long lastSensorReadTime = 0;

unsigned long lastPing = 0;

void loop() {
  if (millis() - lastPing > 10000)
  {
    lastPing = millis();
    webSocket.sendPing();
  }
  // log the sensor stuff once per minute
  if(millis() - lastSensorReadTime > 60000) {
    broadcastSensorData();
    lastSensorReadTime = millis();
  }
  webSocket.loop();


  // reboot every 60 minutes
  if(millis() - bootTime > 3600000) {
    ESP.restart();
  }
}