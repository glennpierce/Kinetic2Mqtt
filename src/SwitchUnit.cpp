#include <PubSubClient.h>
#include "SwitchUnit.h"

// feature toggle
#define ENABLE_HOLD 0  // set to 0 to disable hold/long-press handling

const char* buttonActionToString(ButtonAction action) {
  switch (action) {
    case LEFT_PRESS: return "LEFT_PRESS";
    case RIGHT_PRESS: return "RIGHT_PRESS";
    case RELEASE: return "RELEASE";
#if ENABLE_HOLD
    case LEFT_HOLD: return "LEFT_HOLD";
    case RIGHT_HOLD: return "RIGHT_HOLD";
#endif
    default: return "UNKNOWN";
  }
}

void printActions(const std::vector<ButtonAction>& actions) {
  Serial.print("Actions: [ ");
  for (size_t i = 0; i < actions.size(); ++i) {
    Serial.print(buttonActionToString(actions[i]));
    if (i < actions.size() - 1) {
      Serial.print(", ");
    }
  }
  Serial.println(" ]");
}

SwitchUnit::SwitchUnit()
    : publishMqtt(nullptr), lastAction(RELEASE), lastSentMillis(0),
      isHolding(false), holdStartMillis(0), lastHoldMillis(0),
      holdActionSent(false), holdMessageCount(0)
{
    lastSentSwitchID[0] = '\0';
}

void SwitchUnit::setPublisher(PublishFunc func)
{
    publishMqtt = func;
}

void SwitchUnit::handlePacket(const byte *byteArr, float rssi)
{
    char switchID[5];
    sprintf(switchID, "%02X%02X", byteArr[0], byteArr[1]);

    byte actionByte = byteArr[2];
    std::vector<ButtonAction> actions;

    if ((actionByte & 0b00000001) != 0)
        actions.push_back(LEFT_PRESS);
    if ((actionByte & 0b00001000) != 0)
        actions.push_back(RIGHT_PRESS);
    if (actionByte == 0b11000000)
        actions.push_back(RELEASE);

    printActions(actions);

    unsigned long currentMillis = millis();

    for (ButtonAction action : actions)
    {
        if (strcmp(switchID, lastSentSwitchID) == 0 &&
            action == lastAction &&
            (currentMillis - lastSentMillis < DEBOUNCE_MILLIS))
        {
            delay(10);
            continue; // debounce
        }

        publishRssi(rssi);

#if ENABLE_HOLD
        if (action == LEFT_PRESS || action == RIGHT_PRESS)
        {
            // forcibly clear any previous hold:
            isHolding = false;
            pressPending = false;

            pendingPress = action;
            pressPending = true;
            holdStartMillis = currentMillis;
            lastHoldMillis = currentMillis;
            isHolding = true;
            holdActionSent = false;
            holdMessageCount = 0;
        }

        if (action == RELEASE)
        {
            if (pressPending)
            {
                publish(pendingPress, switchID);
                pressPending = false;
            }

            publish(RELEASE, switchID);
            isHolding = false;
            pendingPress = RELEASE;
        }
#else
        // HOLD disabled, just send press/release directly
        publish(action, switchID);
#endif

        strcpy(lastSentSwitchID, switchID);
        lastAction = action;
        lastSentMillis = currentMillis;
    }
}

void SwitchUnit::checkHold()
{
#if ENABLE_HOLD
    if (isHolding && (millis() - holdStartMillis > HOLD_THRESHOLD_TIME))
    {
        if (pressPending)
        {
            if (pendingPress == LEFT_PRESS)
            {
                Serial.println("Actions: [ LEFT_HOLDING_START ]");
                publish(LEFT_HOLD, lastSentSwitchID);
            }
            else if (pendingPress == RIGHT_PRESS)
            {
                Serial.println("Actions: [ RIGHT_HOLDING_START ]");
                publish(RIGHT_HOLD, lastSentSwitchID);
            }

            pressPending = false; // Don't send the press anymore
        }

        if (holdMessageCount < maxHoldMessages)
        {
            if (millis() - lastHoldMillis >= 1000)
            {
                if (pendingPress == LEFT_PRESS)
                {
                    Serial.printf("Actions: [ LEFT_HOLDING #%d ]\n", holdMessageCount + 1);
                    publish(LEFT_HOLD, lastSentSwitchID);
                }
                else if (pendingPress == RIGHT_PRESS)
                {
                    Serial.printf("Actions: [ RIGHT_HOLDING #%d ]\n", holdMessageCount + 1);
                    publish(RIGHT_HOLD, lastSentSwitchID);
                }

                lastHoldMillis = millis();
                holdActionSent = true;
                holdMessageCount++;
            }
        }
        else
        {
            Serial.println("Actions: [ HOLDING_END ]");
            isHolding = false;
        }
    }
#endif
}

void SwitchUnit::publish(ButtonAction action, const char *switchID)
{
    const char *actionStr = "unknown";
    switch (action)
    {
    case LEFT_PRESS:
        actionStr = "left";
        break;
    case RIGHT_PRESS:
        actionStr = "right";
        break;
    case RELEASE:
        actionStr = "release";
        break;
#if ENABLE_HOLD
    case LEFT_HOLD:
        actionStr = "left_hold";
        break;
    case RIGHT_HOLD:
        actionStr = "right_hold";
        break;
#endif
    }
    publishMqtt("action", switchID, actionStr);
}

void SwitchUnit::publishRssi(float rssi)
{
    char rssiStr[16];
    snprintf(rssiStr, sizeof(rssiStr), "%.1f dBm", rssi);
    publishMqtt("status", "rssi", rssiStr);
}
