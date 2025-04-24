#ifndef DECODER_H
#define DECODER_H

#include <Arduino.h>
#include <RadioLib.h>
#include <vector>
#include <unordered_map>

using PublishFunc = void (*)(const char *, const char *, const char *);

enum ButtonAction
{
  RELEASE,
  LEFT_PRESS,
  RIGHT_PRESS,
  LEFT_HOLD,
  RIGHT_HOLD,
};

class SwitchUnit
{
public:
  SwitchUnit();
  void setPublisher(PublishFunc func);
  void handlePacket(const byte *byteArr, float rssi);
  void checkHold();

private:
  PublishFunc publishMqtt;

  void publish(ButtonAction action, const char *switchID);
  void publishRssi(float rssi);

  ButtonAction pendingPress = RELEASE;
  bool pressPending = false;

  char lastSentSwitchID[5];
  ButtonAction lastAction;
  unsigned long lastSentMillis;

  bool isHolding;
  unsigned long holdStartMillis;
  unsigned long lastHoldMillis;
  bool holdActionSent;
  int holdMessageCount;

  static const unsigned long DEBOUNCE_MILLIS = 500;
  static const unsigned long HOLD_THRESHOLD_TIME = 3000;
  static const int maxHoldMessages = 8;
};

#endif
