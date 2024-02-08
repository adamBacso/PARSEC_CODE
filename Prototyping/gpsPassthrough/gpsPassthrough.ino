void setup(){
    Serial.begin(9600);
    while (!Serial){}
    Serial1.begin(9600);
    while (!Serial1){}
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
}

void loop(){
    if (Serial1.available()){
        Serial.println(Serial2.read());
    }
}