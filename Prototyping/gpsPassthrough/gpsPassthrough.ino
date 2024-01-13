void setup(){
    Serial.begin(9600);
    while (!Serial){}
    Serial2.begin(9600);
    while (!Serial2){}
}

void loop(){
    if (Serial2.available()>0){
        Serial.print(Serial2.read());
    }
}