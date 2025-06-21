#include <PubSubClient.h>
#include "SwitchUnit.h"

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

    // for (int i = 7; i >= 0; i--)
    // {
    //     Serial.print(bitRead(actionByte, i));
    // }
    // Serial.println();

    if ((actionByte & 0b00000001) != 0)
        actions.push_back(LEFT_PRESS);
    if ((actionByte & 0b00001000) != 0)
        actions.push_back(RIGHT_PRESS);
    if (actionByte == 0b11000000)
        actions.push_back(RELEASE);

    unsigned long currentMillis = millis();

    for (ButtonAction action : actions)
    {
        if (strcmp(switchID, lastSentSwitchID) == 0 &&
            action == lastAction &&
            (currentMillis - lastSentMillis < DEBOUNCE_MILLIS))
        {

            continue; // debounce
        }

        publishRssi(rssi);

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

        if (action == LEFT_PRESS || action == RIGHT_PRESS)
        {
            pendingPress = action;
            pressPending = true;
            holdStartMillis = currentMillis;
            lastHoldMillis = currentMillis;
            isHolding = true;
            holdActionSent = false;
            holdMessageCount = 0;
        }

        strcpy(lastSentSwitchID, switchID);
        lastAction = action;
        lastSentMillis = currentMillis;
    }
}

void SwitchUnit::checkHold()
{
    if (isHolding && (millis() - holdStartMillis > HOLD_THRESHOLD_TIME))
    {
        if (pressPending)
        {

            if (pendingPress == LEFT_PRESS)
            {
                publish(LEFT_HOLD, lastSentSwitchID);
            }
            else if (pendingPress == RIGHT_PRESS)
            {
                publish(RIGHT_HOLD, lastSentSwitchID);
            }

            // publish(HOLD, lastSentSwitchID);
            pressPending = false; // Don't send press anymore
        }

        if (holdMessageCount < maxHoldMessages)
        {
            if (millis() - lastHoldMillis >= 1000)
            {

                if (pendingPress == LEFT_PRESS)
                {
                    publish(LEFT_HOLD, lastSentSwitchID);
                }
                else if (pendingPress == RIGHT_PRESS)
                {
                    publish(RIGHT_HOLD, lastSentSwitchID);
                }

                lastHoldMillis = millis();
                holdActionSent = true;
                holdMessageCount++;
            }
        }
        else
        {
            isHolding = false;
        }
    }
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
    case LEFT_HOLD:
        actionStr = "left_hold";
        break;
    case RIGHT_HOLD:
        actionStr = "right_hold";
        break;
    }
    publishMqtt("action", switchID, actionStr);
}

void SwitchUnit::publishRssi(float rssi)
{
    char rssiStr[16];
    snprintf(rssiStr, sizeof(rssiStr), "%.1f dBm", rssi);
    publishMqtt("status", "rssi", rssiStr);
}
