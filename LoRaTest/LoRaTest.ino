#include <LoRa.h>
#include <SPI.h>

void setup(){
    while (!LoRa.begin(868E6)){
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
