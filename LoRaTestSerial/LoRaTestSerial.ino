#define COMMS_SERIAL Serial1 //TX: 1; RX: 0

void setup(){
    COMMS_SERIAL.begin(115200);
    while (!COMMS_SERIAL){
    }
}

int counter = 0;
void loop(){
    COMMS_SERIAL.print("sys get ver\r\n");
    Serial.println(COMMS_SERIAL.read());
    
    delay(500);
}