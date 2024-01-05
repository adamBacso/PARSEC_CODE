void setup(){
    digitalWrite(LED_BUILTIN,HIGH);
    Serial.begin(9600);
    while (!Serial){}
}

void loop(){
    Serial.println("OK");
    delay(500);
}