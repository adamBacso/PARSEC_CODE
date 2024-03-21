void setup(){
    Serial.begin(9600);
    Serial1.begin(9600);
    while (!Serial1);
}

void loop(){
    Serial.println(Serial1.readString());
    delay(250);
}