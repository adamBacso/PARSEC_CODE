#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <FastCRC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <core_pins.h>
#include <usb_serial.h>
#include <math.h>
#include <string>

// SERIAL COMMS
// serial through USB
#define MY_USB_SERIAL   Serial      
// COMMS - serial through pin(1,0) [TX,RX]
#define COMMS_SERIAL    Serial1     
// GPS - serial through pin(8,7) [TX,RX]
#define GPS_SERIAL      Serial2

// BME280
// https://randomnerdtutorials.com/bme280-sensor-arduino-pressure-temperature-humidity/
Adafruit_BME280 bme;
// GLOBALS
const int sdCardChipSelect = 10;
int mode = 0;

class Led {
    public:
        void init(){
            pinMode(LED_BUILTIN,OUTPUT);
            digitalWrite(LED_BUILTIN,LOW);
        }

        void flash(){
            digitalWrite(LED_BUILTIN,HIGH);
            delay(25);
            digitalWrite(LED_BUILTIN,LOW);
        }
};
class MissionTime{
    private:
        unsigned long startTime;
        unsigned int seconds, minutes, hours;

        unsigned long processStart;
    public:
        MissionTime(){
            reset();
        }

        void reset(){
            startTime = millis();
            seconds=0, minutes=0, hours=0;
            processStart = 0;
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

        void true_sleep(int milli){
            delay(milli - (millis()-processStart));
            processStart = millis();
        }
};

MissionTime timer;
class Telemetry{
    /*ID,MISSION_TIME,PACKET_COUNT,TEMPERATURE,BAROMETRIC_ALTITUDE,HUMIDITY,GPS_TIME,GPS_ALTITUDE,GPS_LONGITUDE,GPS_LATITUDE,TILT_X,TILT_Y,TILTZ,ACCELERATION_X,ACCELERATION_Y,ACCELERATION_Z,O3_CONCENTRATION,VOLTAGE,CHECKSUM*/
    private:
        uint16_t ID = 12;
        int packetCount = 0;

        int frequency;
        int commsBaudRate;
        int percentActive;

        const double SEA_LEVEL_PRESSURE_HPA = (1013.25); // FIXME: replace by value true locally

        String format_data(float value, int numberOfBytes){
            if (value == value){
                int digitPlaces;
                digitPlaces = log10f(value) + 1;
                return String(int(value*pow(10,numberOfBytes-digitPlaces)))+",";
            }
            else{
                return "x,";
            }
        }

        String format_data(float value){
            if (value == value){
                return String(value)+",";
            }
            else{
                return "x,";
            }
        }

        String format_data(int value){
            if (value == value){
                return String(value)+",";
            }
            else{
                return "x,";
            }
        }

    public:
        String parse(){ // FIXME: change to send numeric data as opposed to String (String: 1B per character; float: 4B per number)
            String packet = "";
            packet += this->format_data(ID) + ",";
            packet += this->format_data(packetCount);
            packet += timer.get_time() + ",";
            /*
            packet += this->format_data(bme.readTemperature());
            packet += this->format_data(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA));
            packet += this->format_data(bme.readHumidity()); // TODO: add readings for all components
            */
            packetCount=packetCount+1;
            return packet + this->get_checksum(packet);
        }

        void init(){
            packetCount = 0;
            timer.reset();
            this->send("Telemetry INIT: OK");
        }

        String get_checksum(String data){
                    byte checksum = 0;
                    for (char ch : data){
                        checksum ^= ch;
                    }
                    return String(checksum,HEX);
                }

        void send(){
            if (mode == 0){
                if (COMMS_SERIAL.availableForWrite()>0){
                    COMMS_SERIAL.println(this->parse());
                }
            }
            else {
                if (MY_USB_SERIAL.availableForWrite()>0){
                    MY_USB_SERIAL.println(this->parse());
                }
            }
        }

        

        void send(const String& message){
            if (mode == 0){
                if (COMMS_SERIAL.availableForWrite()>0){
                    COMMS_SERIAL.println(message);
                }
            }
            else {
                if (MY_USB_SERIAL.availableForWrite()>0){
                    MY_USB_SERIAL.println(message);
                }
            }

            
        }

        void write(){
            if (SD.begin(sdCardChipSelect)){
                File logFile = SD.open("logFile.txt", FILE_WRITE);
                if (logFile){
                    logFile.println(this->parse());
                }
                logFile.close();
            }
        }

        
        void sd_validate(){        
            //if (!SD.begin(chipSelect)){
            //    // TODO: indicate that no SD card
            //    while (1){
            //        // stop here
            //    }
            //}
        }

        void communicate(int mode = 0){
            this->write();
            this->send();
        }
};



Telemetry data;
Led led;


void setup(){
    
    MY_USB_SERIAL.begin(9600);
    // TODO: add component validation
    data.sd_validate();
    data.init();
    led.init();
    mode = 1;
    led.flash();
}

void loop(){
    //data.send(1);
    data.send();
    led.flash();
    timer.true_sleep(1000);
}