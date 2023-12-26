---
title: Serial Communication
---
This is how the serial communication works in the program.

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="15">

---

Defining the serial ports used for communicating and for gps.

```arduino
// SERIAL COMMS  
// COMMS - **Serial1** through pin(1,0) [TX,RX]      // USB SERIAL - **Serial** through USB port
#define COMMS_SERIAL    Serial  // FIXME: change to Serial1 on final version
// GPS - serial through pin(8,7) [TX,RX]
#define GPS_SERIAL      Serial2
```

---

</SwmSnippet>

Communication is handled by the <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="122:2:2" line-data="class Telemetry{">`Telemetry`</SwmToken> class. In the main ,  is called. This checks if  <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="125:3:3" line-data="        bool broadcasting = false;">`broadcasting`</SwmToken> is `True` and wether or not the program has waited enough to broadcast yet again. Printing through the serial port is done on a component-by-component basis. If a component is unavailable, the `0` is passed in as its value.

<SwmSnippet path="Flight_Controller/Flight_Controller.ino" line="221">

---

{Values are only read from the BME280 if it was initialized. Otherwise, its values are added az zeros.

```
            // BME280
            if (bme.begin()){
                this->prints(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA));
                this->prints(bme.readTemperature());
                this->prints(bme.readHumidity());
            }else{
                for (int i = 0; i<3; i++){
                    this->prints(0);
                }
            }
```

---

</SwmSnippet>

## Sleeping

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="276">

---

Sleeping for the amount of time not active minus the elapsed process time

```
        void set_sleep_amount(uint32_t elapsedTime){
            uint32_t transmissionTime = (bitSize / commsBaudRate) * 1000;
            elapsedTime -= transmissionTime;
            sleepAmount = (1-percentActive)*10 * transmissionTime - elapsedTime;
        }
```

---

</SwmSnippet>

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="277">

---

Calculate the time it takes to transmit  data of <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="143:3:3" line-data="        uint32_t bitSize;">`bitSize`</SwmToken> bits through a Serial port with a baud rate of <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="132:3:3" line-data="        int commsBaudRate = 9600;">`commsBaudRate`</SwmToken> bps.

```arduino
            uint32_t transmissionTime = (bitSize / commsBaudRate) * 1000;
```

---

</SwmSnippet>

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="278">

---

Subtract transmission time as we are interested in the externalities

```arduino
            elapsedTime -= transmissionTime;
```

---

</SwmSnippet>

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="279">

---

Calculate the time when not active and subtract the elapsed time

```arduino
            sleepAmount = (1-percentActive)*10 * transmissionTime - elapsedTime;
```

---

</SwmSnippet>

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBUEFSU0VDX0NPREUlM0ElM0FhZGFtQmFjc28=" repo-name="PARSEC_CODE"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
