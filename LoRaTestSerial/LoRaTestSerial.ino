#define COMMS_SERIAL    Serial1 

// #include <HardwareSerial.h>
#include <SPI.h>


void setup(){
    COMMS_SERIAL.begin(115200);
    while (!COMMS_SERIAL){
    }
    Serial.begin(9600);
}

int counter = 0;
void loop(){
    // Check if data is available on USB Serial
  while (Serial.available()) {
    String data = Serial.readString();
    COMMS_SERIAL.println(data); // Send data to Software Serial

    //Serial.flush() //is very slow
  }

  // Check if data is available on Software Serial
  while (COMMS_SERIAL.available()) {
    String data = COMMS_SERIAL.readString();
    Serial.println(data); // Send data to USB Serial

    //LoRaSerial.flush();
  }
}