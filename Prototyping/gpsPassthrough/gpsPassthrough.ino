void setup(){
    Serial.begin(9600);
    Serial1.begin(9600);
    while (!Serial1);
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,HIGH);
}

void loop(){
    Serial.println(Serial1.readString());
    delay(250);
}