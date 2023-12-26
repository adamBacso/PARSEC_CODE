---
title: Wiring
---
Here is a guide on how to wire components and where to change pin-s if neccessary.

## Serial communication

The serial pins of Teensy 4.0 is as follows:

| Serial number | Pins (TX, RX) |
| ------------- | ------------- |
| Serial        | USB           |
| Serial1       | 1, 0          |
| Serial2       | 8, 7          |
| Serial3       | 14, 15        |
| Serial4       | 17, 16        |
| Serial5       | 20, 21        |
| Serial6       | 24, 25        |
| Serial7       | 29, 28        |

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="15">

---

Serial ports can be set here

```arduino
// SERIAL COMMS  
// COMMS - **Serial1** through pin(1,0) [TX,RX]      // USB SERIAL - **Serial** through USB port
#define COMMS_SERIAL    Serial  // FIXME: change to Serial1 on final version
// GPS - serial through pin(8,7) [TX,RX]
#define GPS_SERIAL      Serial2
```

---

</SwmSnippet>

## I2C components

- BME280
- MPU-6050

Wiring to the right pin is not that important, components are accessed through their adresses. Only two wires are necessary, an SCL (pin 19) and SDA (pin 18). \*\*Note: there are other pins available, these are just examples. For the others, see the pin layout in [Electrical Systems](https://docs.google.com/document/d/16jo1tMiNq1_cbLjWFedkwZh4niLgg9v6KwrCJ_M0tjk/edit?usp=sharing) document.

## GUVA-S12SD

This component uses <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="36:7:7" line-data="            float sensorValue = analogRead(analogPin);">`analogRead`</SwmToken> to read the voltage. To change the input pin or the operating voltage, change the following:

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="140">

---

guva initialization

```arduino
        LightSensor guva = LightSensor(14,3.3);
```

---

</SwmSnippet>

<SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="33:1:1" line-data="        LightSensor(int pin, float voltage) : analogPin(pin), operatingVoltage(voltage){}">`LightSensor`</SwmToken> takes the parameters <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="29:3:3" line-data="        int analogPin;">`analogPin`</SwmToken> and <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="30:3:3" line-data="        float operatingVoltage;">`operatingVoltage`</SwmToken>.

## microSD Card Reader

The following pins must be connected:

| Name | pin    |
| ---- | ------ |
| MISO | 12, 34 |
| MOSI | 11, 35 |
| SCK  | 13, 37 |
| CS   | 10, 36 |

The program only cares about the <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="43:5:5" line-data="       const int SD_CARD_CHIP_SELECT;">`SD_CARD_CHIP_SELECT`</SwmToken>

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="144">

---

SD card reader initialization

```arduino
        SDCard sd = SDCard(10);
```

---

</SwmSnippet>

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBUEFSU0VDX0NPREUlM0ElM0FhZGFtQmFjc28=" repo-name="PARSEC_CODE"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
