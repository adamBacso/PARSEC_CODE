#include <TinyGPSPlus.h>
#include <usb_serial.h>

TinyGPSPlus gps;
void setup(){
    Serial.begin(9600);
    while(!Serial){}
    Serial2.begin(9600);
    while(!Serial2){}
}

void loop(){
    Serial.print("lat:");
    Serial.print(gps.location.lat());
    Serial.print(";\t\tlng: ");
    Serial.println(gps.location.lng());

    unsigned long start = millis();
    do 
    {
    while (Serial2.available())
        gps.encode(Serial2.read());
    } while (millis() - start < 1000); // 1 second delay
}