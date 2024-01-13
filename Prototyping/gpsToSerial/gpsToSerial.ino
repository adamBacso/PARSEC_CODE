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

    unsigned long start = millis();
    do 
    {
    while (Serial2.available()>0)
        if(gps.encode(Serial2.read())){
            displayInfo();
        };
    } while (millis() - start < 1000); // 1 second delay
}

void displayInfo(){
    Serial.print("lat:");
    Serial.print(gps.location.lat());
    Serial.print(";\t\tlng: ");
    Serial.println(gps.location.lng());
}