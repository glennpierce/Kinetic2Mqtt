// publishes
// kinetic_switches/status/rssi: -20.000000 dBm
// kinetic_switches/action/1E6B: press
// kinetic_switches/status/rssi: -19.000000 dBm
// kinetic_switches/action/1E6B: release
// kinetic_switches/action/1E6B: hold
// kinetic_switches/action/1E6B: hold

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <RadioLib.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include <PubSubClient.h>
#include <AceCRC.h>
#include <string>
#include "SwitchUnit.h"
#include "Kinetic2MQTT.h"

// CRC-16/SPI-FUJITSU
// CRC-16/AUG-CCITT
// Select the type of CRC algorithm we'll be using
using namespace ace_crc::crc16ccitt_byte;

// Radio Config
#define PACKET_LENGTH 5           // bytes
#define MIN_RSSI -90              // dBm
#define CARRIER_FREQUENCY 433.3   // MHz
#define BIT_RATE 100.0            // kbps
#define FREQUENCY_DEVIATION 50.0  // kHz

// #define RX_BANDWIDTH 150.0     // kH
#define RX_BANDWIDTH 400.0        // kH

#define OUTPUT_POWER 10           // dBm
#define PREAMBLE_LENGTH 16        // bits

// IotWebConf Config
#define CONFIG_PARAM_MAX_LEN 128
#define CONFIG_VERSION "mqt2"
#define CONFIG_PIN 4  // D2
#define STATUS_PIN 16 // D0

// Kinetic2MQTT Config
#define DEBOUNCE_MILLIS 15

#define HOLD_THRESHOLD_TIME 800

// CC1101 has the following connections:
// CS pin:    15
// GDO0 pin:  5
// RST pin:   unused
// GDO2 pin:  unused (optional)
CC1101 radio = new Module(15, 5, RADIOLIB_NC, RADIOLIB_NC);

// Set up IotWebConf
const char deviceName[] = "kinetic2mqtt";
const char apPassword[] = "EMh896WhP56Q"; // Default password for SSID and configuration page, can be changed after first boot

void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper);

DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
PubSubClient mqttClient(net);

char mqttServerValue[CONFIG_PARAM_MAX_LEN];
// char mqttUserNameValue[CONFIG_PARAM_MAX_LEN];
// char mqttUserPasswordValue[CONFIG_PARAM_MAX_LEN];
char mqttTopicValue[CONFIG_PARAM_MAX_LEN];

IotWebConf iotWebConf(deviceName, &dnsServer, &server, apPassword, CONFIG_VERSION);
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, CONFIG_PARAM_MAX_LEN);
// IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, CONFIG_PARAM_MAX_LEN);
// IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, CONFIG_PARAM_MAX_LEN);
IotWebConfTextParameter mqttTopicParam = IotWebConfTextParameter("MQTT Topic", "mqttTopic", mqttTopicValue, CONFIG_PARAM_MAX_LEN);

bool needReset = false;

//std::unordered_map<String, SwitchUnit> switches;
std::unordered_map<std::string, SwitchUnit> switches;

// Convert array of bytes into a string containing the HEX representation of the array
static void bytesToHexString(byte array[], unsigned int len, char buffer[])
{
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
    buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
  }
  buffer[len * 2] = '\0';
}

// Publish [value] to MQTT at topic: [mqttTopicValue]/[topicLevel1]/[topicLevel2]
static void publishMqtt(PubSubClient& mqttClient, const char *topicLevel0, const char *topicLevel1, const char *topicLevel2, const char *value)
{
  char compiledTopic[strlen(topicLevel0) + strlen(topicLevel1) + strlen(topicLevel2) + 3] = "";
  strcat(compiledTopic, topicLevel0);
  strcat(compiledTopic, "/");
  strcat(compiledTopic, topicLevel1);
  strcat(compiledTopic, "/");
  strcat(compiledTopic, topicLevel2);

  Serial.print("publishMqtt: ");
  Serial.print(compiledTopic);
  Serial.print(", value: ");
  Serial.println(value);

  mqttClient.publish(compiledTopic, value);
}

void setup()
{
  Serial.begin(115200);
  pinMode(CONFIG_PIN, INPUT_PULLUP);

  // Initialise and configure CC1101

  Serial.print("Initializing Radio... ");

  int state = radio.begin(CARRIER_FREQUENCY, BIT_RATE, FREQUENCY_DEVIATION, RX_BANDWIDTH, OUTPUT_POWER, PREAMBLE_LENGTH);
  radio.setCrcFiltering(false);
  radio.fixedPacketLengthMode(PACKET_LENGTH);

  uint8_t syncWord[] = {0xA4, 0x23};
  radio.setSyncWord(syncWord, 2);

  radio.setGdo0Action(setFlag, RISING);

  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println("done!");
  }
  else
  {
    Serial.print("failed, code ");
    Serial.println(state);
    while (true)
      ;
  }

  // Initialise IOTWebConf

  Serial.println("Initialising IotWebConf... ");
  mqttGroup.addItem(&mqttServerParam);
  // mqttGroup.addItem(&mqttUserNameParam);
  // mqttGroup.addItem(&mqttUserPasswordParam);
  mqttGroup.addItem(&mqttTopicParam);

  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addParameterGroup(&mqttGroup);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);

  bool validConfig = iotWebConf.init();
  if (!validConfig)
  {
    mqttServerValue[0] = '\0';
    // mqttUserNameValue[0] = '\0';
    // mqttUserPasswordValue[0] = '\0';
    mqttTopicValue[0] = '\0';
  }

  // This will disable the device sitting in AP mode for 30s on startup
  // Not requred due to presence of reset button to manually enable AP mode
  iotWebConf.setApTimeoutMs(0);

  server.on("/", []
            { iotWebConf.handleConfig(); });
  server.onNotFound([]()
                    { iotWebConf.handleNotFound(); });

  // Initialise MQTT

  mqttClient.setServer(mqttServerValue, 1883);

  // Initialize OTA
  //ArduinoOTA.setHostname("kinetic2mqtt");
  //ArduinoOTA.begin();
}

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif

void myMqttPublish(const char* topic1, const char* topic2, const char* value) {  

  publishMqtt(mqttClient, mqttTopicValue, topic1, topic2, value);
}

void setFlag(void)
{
  // we got a packet, set the flag
  // Serial.println("Received Packet");
  receivedFlag = true;
}

void loop()
{
  iotWebConf.doLoop();

  if (needReset)
  {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }

  // If WiFi is connected but MQTT is not, establish MQTT connection
  if ((iotWebConf.getState() == iotwebconf::OnLine) && !mqttClient.connected())
  {
    connectMqtt();
  }
  mqttClient.loop();

  // If a message has been received and flag has been set by interrupt, process the message
  if (receivedFlag)
  {
    byte byteArr[PACKET_LENGTH];
    int state = radio.readData(byteArr, PACKET_LENGTH);

    if (state == RADIOLIB_ERR_NONE)
    {
      if (radio.getRSSI() > MIN_RSSI)
      {
       
        byte messageArr[] = {byteArr[0], byteArr[1], byteArr[2]};
        unsigned long messageCRC = (byteArr[3] << 8) | byteArr[4];
        crc_t crc = crc_init();
        crc = crc_update(crc, messageArr, 3);
        crc = crc_finalize(crc);

        if (crc == messageCRC) {
          char switchID[5];
          bytesToHexString(byteArr, 2, switchID);
          //String idStr(switchID);
          std::string idStr(switchID);
    
          if (switches.find(idStr) == switches.end()) {
            switches[idStr] = SwitchUnit();
            switches[idStr].setPublisher(myMqttPublish);
          }
    
          switches[idStr].handlePacket(byteArr, radio.getRSSI());

        } else {
          Serial.println("CRC mismatch.");
        }
      }
    }
    else
    {
      Serial.print("RadioLib error: ");
      Serial.println(state);
    }

    receivedFlag = false;
    radio.startReceive();
  }

  for (auto& pair : switches) {
    pair.second.checkHold();
  }

  // Handle OTA updates
  // ArduinoOTA.handle();

  delay(50); // Add a 50ms delay
}

// Establish an MQTT connection, if connection fails this function will delay for 5 seconds and then return
// Connection will be re-attempted at next execution of loop()
void connectMqtt()
{
  // Loop until we're reconnected
  Serial.print("Attempting MQTT connection...");
  // Create a random client ID
  String clientId = iotWebConf.getThingName();
  clientId += "-";
  clientId += String(random(0xffff), HEX);
  // Attempt to connect
  if (mqttClient.connect(clientId.c_str()))
  {
    Serial.println("connected");
    // Once connected, publish an announcement...
    char topicLevel1[] = "status";
    char topicLevel2[] = "connection";
    char messageValue[] = "connected";
    publishMqtt(mqttClient, mqttTopicValue, topicLevel1, topicLevel2, messageValue);
  }
  else
  {
    Serial.print("MQTT Connection Failed:");
    Serial.print(mqttClient.state());
    Serial.println(".  Trying again in 5 seconds");
    // Wait 5 seconds before retrying
    iotWebConf.delay(5000);
  }
}

// If configuration is saved in IOTWebConf, reboot the device
void configSaved()
{
  Serial.println("Configuration was updated.");
  needReset = true;
}

// Validate the data entered into the IOTWebConf configuration page
bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper)
{
  Serial.println("Validating form.");
  bool valid = true;

  int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
  if (l < 3)
  {
    mqttServerParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  l = webRequestWrapper->arg(mqttTopicParam.getId()).length();
  if (l < 3)
  {
    mqttTopicParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  return valid;
}
