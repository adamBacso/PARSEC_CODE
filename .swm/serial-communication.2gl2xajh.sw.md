---
title: Serial Communication
---
This is how the serial communication works in the program.

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="13">

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

Communication is handled by the <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="81:2:2" line-data="class Telemetry{">`Telemetry`</SwmToken> class. In the main ,  is called. This checks if  <SwmToken path="/Flight_Controller/Flight_Controller.ino" pos="84:3:3" line-data="        bool broadcasting = false;">`broadcasting`</SwmToken> is `True` and wether or not the program has waited enough to broadcast yet again. Printing through the serial port is done on a component-by-component basis. If a component is unavailable, the `0` is passed in as its value.

<SwmSnippet path="/Flight_Controller/Flight_Controller.ino" line="148">

---

Values are only read from the BME280 if it was initialized. Otherwise, its values are added az zeros.

```arduino
            // BME280
            if (bme.begin()){
                this->prints(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA), dataIndex++);
                this->prints(bme.readTemperature(), dataIndex++);
                this->prints(bme.readHumidity(), dataIndex++);
            }else{
                this->prints(0, dataIndex++);
                this->prints(0, dataIndex++);
                this->prints(0, dataIndex++);
            }
```

---

</SwmSnippet>

<SwmMeta version="3.0.0" repo-id="Z2l0aHViJTNBJTNBUEFSU0VDX0NPREUlM0ElM0FhZGFtQmFjc28=" repo-name="PARSEC_CODE"><sup>Powered by [Swimm](https://app.swimm.io/)</sup></SwmMeta>
