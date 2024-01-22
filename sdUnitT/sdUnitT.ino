#include <SD.h>
#include <SPI.h>

class mySD{
    private:
        const int chipSelect = 14;
        File flightLog;
        const char* logName = "flightLog.csv";

    public:

        int get_chipSelect(){
            return chipSelect;
        }

        void begin(){
            if (SD.begin(chipSelect)){
                flightLog = SD.open(logName, FILE_WRITE);
                if (flightLog){
                    flightLog.println("~~~~~~ SYSTEM RESTART ~~~~~~");
                }
                flightLog.close();
            }
        }

        void writeln(String data){
            if (SD.begin(chipSelect)){
                flightLog = SD.open(logName, FILE_WRITE);
                if (flightLog){
                    flightLog.println(data.c_str());
                }
                flightLog.close();
            }
        }

};

mySD sd;

void setup(){
    Serial.begin(9600);
    while (!Serial);
    sd.begin();
}

void loop(){
    String data = millis() + " : OK";
    sd.writeln(data);
    Serial.println(data);
    delay(1000);
}