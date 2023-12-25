---
title: Serial Communication
---
This is how the serial communication works in the program.

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="14">

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

Communication is handled by the <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="83:2:2" line-data="class Telemetry{">`Telemetry`</SwmToken> class. In the main ,  is called. This checks if  <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="86:3:3" line-data="        bool broadcasting = false;">`broadcasting`</SwmToken> is `True` and wether or not the program has waited enough to broadcast yet again. Printing through the serial port is done on a component-by-component basis. If a component is unavailable, the `0` is passed in as its value.

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="171">

---

{Values are only read from the BME280 if it was initialized. Otherwise, its values are added az zeros.

```arduino
            // BME280
            if (bme.begin()){
                this->prints(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA));
                this->prints(bme.readTemperature());
                this->prints(bme.readHumidity());
            }else{
                this->prints(0);
                this->prints(0);
                this->prints(0);
            }
```

---

</SwmSnippet>

## Sleeping

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="203">

---

sleeps for the remainder of a second

```arduino
        void set_sleep_amount(){
            sleepAmount = 1000 - (bitSize / commsBaudRate)*1000;
        }
```

---

</SwmSnippet>

<SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="100:5:5" line-data="        unsigned long bitSize;">`bitSize`</SwmToken>  /  <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="93:3:3" line-data="        int commsBaudRate = 9600;">`commsBaudRate`</SwmToken>  = time it takes to transmit a message (in seconds)

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBUEFSU0VDX0NPREUlM0ElM0FhZGFtQmFjc28=" repo-name="PARSEC_CODE"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
