#include <LoRa.h>
#include <SPI.h>

#define COMMS_SERIAL Serial

void setup(){
    COMMS_SERIAL.begin(9600);
    while (!LoRa.begin(868E6)){
        COMMS_SERIAL.println("Failed RF gateway");
        delay(500);
    }
    LoRa.setSyncWord(12); // set HTerm sync to the same!!
}

int counter = 0;
void loop(){
    LoRa.beginPacket();
    LoRa.print("Hello! ");
    LoRa.print(counter++);
    LoRa.endPacket();
    delay(1000);
}
