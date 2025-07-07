#include <Arduino.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <PubSubClient.h>
#include <AceCRC.h>
#include <unordered_map>
#include <string>
#include "SwitchUnit.h"
#include <CircularBuffer.h>

// CRC helper
using namespace ace_crc::crc16ccitt_byte;

// --- Feature toggles ---
#define ENABLE_MQTT 1 // <-- set to 0 to disable MQTT send

// --- Radio Config ---
#define PACKET_LENGTH 5
#define MIN_RSSI -105
#define CARRIER_FREQUENCY 433.3
#define BIT_RATE 100.0
#define FREQUENCY_DEVIATION 50.0
#define RX_BANDWIDTH 135.0
#define OUTPUT_POWER 10
#define PREAMBLE_LENGTH 16
#define PACKET_QUEUE_SIZE 8

// CC1101 module
CC1101 radio = new Module(15, 5, RADIOLIB_NC, RADIOLIB_NC);

// --- Pins ---
#define CONFIG_PIN 4
#define STATUS_PIN 16
const int MQTT_SEND_PIN = 25;
const int ERROR_PIN = 26;

// --- IotWebConf ---
#define CONFIG_PARAM_MAX_LEN 128
#define CONFIG_VERSION "mqt2"

char mqttServerValue[CONFIG_PARAM_MAX_LEN];
char mqttTopicValue[CONFIG_PARAM_MAX_LEN];

DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
PubSubClient mqttClient(net);

IotWebConf iotWebConf("kinetic2mqtt", &dnsServer, &server, "EMh896WhP56Q", CONFIG_VERSION);
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, CONFIG_PARAM_MAX_LEN);
IotWebConfTextParameter mqttTopicParam = IotWebConfTextParameter("MQTT Topic", "mqttTopic", mqttTopicValue, CONFIG_PARAM_MAX_LEN);

bool needReset = false;

struct Packet {
  uint8_t data[PACKET_LENGTH];
  float rssi;
};

CircularBuffer<Packet, PACKET_QUEUE_SIZE> packetQueue;

TaskHandle_t radioTaskHandle = NULL;

// --- Switch state ---
std::unordered_map<std::string, SwitchUnit> switches;

// For simple debounce
Packet lastPacket;
unsigned long lastPacketTime = 0;

// --- Utils ---
static void bytesToHexString(byte array[], unsigned int len, char buffer[]) {
  for (unsigned int i = 0; i < len; i++) {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
    buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
  }
  buffer[len * 2] = '\0';
}

static void publishMqtt(PubSubClient& mqttClient, const char *topicLevel0, const char *topicLevel1, const char *topicLevel2, const char *value) {
#if ENABLE_MQTT
  char compiledTopic[strlen(topicLevel0) + strlen(topicLevel1) + strlen(topicLevel2) + 3] = "";
  strcat(compiledTopic, topicLevel0);
  strcat(compiledTopic, "/");
  strcat(compiledTopic, topicLevel1);
  strcat(compiledTopic, "/");
  strcat(compiledTopic, topicLevel2);

  mqttClient.publish(compiledTopic, value);

  digitalWrite(MQTT_SEND_PIN, HIGH);
  delay(50);
  digitalWrite(MQTT_SEND_PIN, LOW);
#else
  Serial.printf("[INFO] MQTT disabled: %s/%s/%s -> %s\n",
    topicLevel0, topicLevel1, topicLevel2, value);
#endif
}

// --- ISR ---
void IRAM_ATTR setFlag() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(radioTaskHandle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// --- Fast radio task ---
void radioTask(void *pvParameters) {
  Packet incoming;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    int state = radio.readData(incoming.data, PACKET_LENGTH);
    incoming.rssi = radio.getRSSI();

    if (state == RADIOLIB_ERR_NONE) {
      if (incoming.rssi >= MIN_RSSI) {
        unsigned long now = millis();
        bool isDuplicate = true;
        for (int i = 0; i < PACKET_LENGTH; i++) {
          if (incoming.data[i] != lastPacket.data[i]) {
            isDuplicate = false;
            break;
          }
        }

        if (isDuplicate && (now - lastPacketTime) < 50) {
          // debounced duplicate
        } else {
          lastPacket = incoming;
          lastPacketTime = now;
          packetQueue.push(incoming);
        }
      } else {
        // log dropped weak packet
        // char hexBuffer[PACKET_LENGTH * 2 + 1];
        // bytesToHexString(incoming.data, PACKET_LENGTH, hexBuffer);
        // Serial.printf("[%lu ms] [INFO] Packet dropped due to low RSSI: %s, RSSI: %.1f\n",
        //               millis(), hexBuffer, incoming.rssi);
      }
    }
    radio.startReceive();
  }
}

// --- MQTT helper ---
void myMqttPublish(const char* topic1, const char* topic2, const char* value) {
  publishMqtt(mqttClient, mqttTopicValue, topic1, topic2, value);
}

// --- IotWebConf ---
void configSaved() {
  Serial.println("Configuration updated.");
  needReset = true;
}

bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper) {
  bool valid = true;
  int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
  if (l < 3) {
    mqttServerParam.errorMessage = "Provide at least 3 chars!";
    valid = false;
  }
  l = webRequestWrapper->arg(mqttTopicParam.getId()).length();
  if (l < 3) {
    mqttTopicParam.errorMessage = "Provide at least 3 chars!";
    valid = false;
  }
  return valid;
}

void connectMqtt() {
#if ENABLE_MQTT
  Serial.print("Connecting MQTT...");
  String clientId = iotWebConf.getThingName() + String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("connected");
    publishMqtt(mqttClient, mqttTopicValue, "status", "connection", "connected");
  } else {
    Serial.printf("MQTT failed (%d), retrying in 5s\n", mqttClient.state());
    delay(5000);
  }
#else
  Serial.println("MQTT connect disabled (ENABLE_MQTT=0)");
#endif
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  pinMode(CONFIG_PIN, INPUT_PULLUP);
  pinMode(MQTT_SEND_PIN, OUTPUT);
  pinMode(ERROR_PIN, OUTPUT);

  xTaskCreatePinnedToCore(radioTask, "RadioTask", 4096, NULL, 2, &radioTaskHandle, 1);

  SPI.begin(18, 19, 23, 15);

  Serial.println("Initializing Radio...");
  int state = radio.begin(CARRIER_FREQUENCY, BIT_RATE, FREQUENCY_DEVIATION, RX_BANDWIDTH, OUTPUT_POWER, PREAMBLE_LENGTH);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("Radio init failed: %d\n", state);
    digitalWrite(ERROR_PIN, HIGH);
    while (1);
  }

  radio.setCrcFiltering(false);
  radio.fixedPacketLengthMode(PACKET_LENGTH);
  radio.setPreambleLength(16, 1);
  uint8_t syncWord[] = {0xA4, 0x23};
  radio.setSyncWord(syncWord, 2);

  radio.setGdo0Action(setFlag, RISING);
  radio.startReceive();

  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttTopicParam);
  iotWebConf.addParameterGroup(&mqttGroup);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.init();
  iotWebConf.setApTimeoutMs(0);

  server.on("/", []() { iotWebConf.handleConfig(); });
  server.onNotFound([]() { iotWebConf.handleNotFound(); });

#if ENABLE_MQTT
  mqttClient.setServer(mqttServerValue, 1883);
#endif
}

// --- Loop ---
void loop() {
  iotWebConf.doLoop();

  if (needReset) {
    delay(1000);
    ESP.restart();
  }

#if ENABLE_MQTT
  if (iotWebConf.getState() == iotwebconf::OnLine && !mqttClient.connected()) {
    connectMqtt();
  }
  mqttClient.loop();
#endif

  while (!packetQueue.isEmpty()) {
    Packet p = packetQueue.shift();

    byte messageArr[] = {p.data[0], p.data[1], p.data[2]};
    unsigned long messageCRC = (p.data[3] << 8) | p.data[4];
    crc_t crc = crc_init();
    crc = crc_update(crc, messageArr, 3);
    crc = crc_finalize(crc);

    if (crc == messageCRC) {
      char switchID[5];
      bytesToHexString(p.data, 2, switchID);
      std::string idStr(switchID);

      if (switches.find(idStr) == switches.end()) {
        switches[idStr] = SwitchUnit();
        switches[idStr].setPublisher(myMqttPublish);
      }
      switches[idStr].handlePacket(p.data, p.rssi);
    } else {
      char hexBuffer[PACKET_LENGTH * 2 + 1];
      bytesToHexString(p.data, PACKET_LENGTH, hexBuffer);
      Serial.printf("[%lu ms] [WARN] CRC mismatch on packet: %s, RSSI: %.1f\n",
                    millis(), hexBuffer, p.rssi);
    }
  }

  for (auto& pair : switches) {
    pair.second.checkHold();
  }

  delay(20);
}
