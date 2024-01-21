#include <usb_serial.h>

bool ledState = true;
void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN,ledState);
    Serial.begin(9600);
    while (!Serial){}
}

int counter = 0;
void loop(){
    Serial.println(counter++);
    delay(500);
    ledState = !ledState;
    digitalWrite(LED_BUILTIN,ledState);
}