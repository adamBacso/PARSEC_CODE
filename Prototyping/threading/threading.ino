#include <usb_serial.h>
#include <TeensyThreads.h>

void blink_thread(){
    while (1){
        digitalWrite(LED_BUILTIN,HIGH);
        threads.delay(25);
        digitalWrite(LED_BUILTIN,LOW);
        threads.delay(975);
        threads.yield();
    }
}

void heartbeat(){
    int count = 0;
    while (1){
        Serial.println(count + "HELLO");
        threads.delay(500);
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