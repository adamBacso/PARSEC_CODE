#include <usb_serial.h>
#include <TeensyThreads.h>

void blink_thread(){
    while (1){
        digitalWrite(LED_BUILTIN,HIGH);
        threads.delay(500);
        digitalWrite(LED_BUILTIN,LOW);
        threads.delay(500);
        threads.yield();
    }
}

void heartbeat(){
    while (1){
        Serial.println(millis() + " - HELLO");
        threads.delay(750);
        threads.yield();
    }
}

void setup(){
    Serial.begin(9600);
    while(!Serial){}
    threads.addThread(blink_thread);
    threads.addThread(heartbeat);
}

void loop(){

}