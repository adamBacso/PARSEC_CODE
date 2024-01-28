int currentAngle;
int read_potentiometer(){
    int analogValue = analogRead(A0);
    int angle = map(analogValue, 0, 1023, 0, 180, 0);
    return angle;
}

String string_to_hex(String data){
    String hexString = "";

    for (unsigned int i = 0; i < data.length(); i++) {
        char currentChar = data.charAt(i);
        char hexChars[3];
        sprintf(hexChars, "%02X", currentChar);
        hexString += hexChars;
    }
    
    return hexString;
}

String string_to_hex(int numericData){
    String data = String(numericData);
    String hexString = "";

    for (unsigned int i = 0; i < data.length(); i++) {
        char currentChar = data.charAt(i);
        char hexChars[3];
        sprintf(hexChars, "%02X", currentChar);
        hexString += hexChars;
    }
    
    return hexString;
}


void setup(){
    Serial.begin(9600);
    Serial1.begin(115200);
    currentAngle = read_potentiometer();
    pinMode(A0, INPUT);
}

void loop(){
    if (currentAngle != read_potentiometer()){
        Serial1.println("radio tx " + string_to_hex(read_potentiometer()) + " 1");
    }
    delay(800);
}