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
    if (COMMS_SERIAL.available()>0){
        Serial.println(COMMS_SERIAL.read(),DEC);
    }
    else{
        COMMS_SERIAL.println("sys get ver\r\n");
        delay(500);
    }
}