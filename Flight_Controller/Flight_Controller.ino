#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// SERIAL COMMS
#define USB_SERIAL              Serial      // serial through USB
#define COMMS_SERIAL            Serial1     // serial through pin(1,0) [TX,RX]
#define GPS_SERIAL              Serial2     // serial through pin(8,7) [TX,RX]

// BME280
Adafruit_BME280 bme;
const double SEALEVELPRESSURE_HPA = (1013.25); // FIXME: replace by value true locally


void setup(){
    USB_SERIAL.begin(9600);
    MissionTime timer;
}

void loop(){

}

class Telemetry{
    /*ID,MISSION_TIME,TEMPERATURE,BAROMETRIC_ALTITUDE,HUMIDITY,GPS_ALTITUDE,GPS_LONGITUDE,GPS_LATITUDE,TILT_X,TILT_Y,TILTZ,ACCELERATION_X,ACCELERATION_Y,ACCELERATION_Z,O3_CONCENTRATION,VOLTAGE,CHECKSUM*/
    private:
        const String ID = "12";
        int packetCount;
    public:
        String createPacket(){
        }
};

class MissionTime{
    private:
        unsigned long startTime;
        static unsigned int seconds, minutes, hours;
    public:
        MissionTime(){
            reset();
        }

        void reset(){
            startTime = millis();
            seconds=0, minutes=0, hours=0;
        }

        String get_time(){
            unsigned long elapsedTime = (millis()-startTime)/1000; // in seconds

            seconds = elapsedTime % 60;
            minutes = (elapsedTime % 3600) / 60;
            hours = elapsedTime / 3600;

            char buffer[9]; // HH:MM:SS0
            snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", hours, minutes, seconds);

            return String(buffer);
        }
};