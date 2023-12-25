#include <LoRa.h>
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
// COMMS - **Serial1** through pin(1,0) [TX,RX]      // USB SERIAL - **Serial** through USB port
#define COMMS_SERIAL    Serial  // FIXME: change to Serial1 on final version
// GPS - serial through pin(8,7) [TX,RX]
#define GPS_SERIAL      Serial2



// BME280
// https://randomnerdtutorials.com/bme280-sensor-arduino-pressure-temperature-humidity/
Adafruit_BME280 bme;
// GLOBALS
const int SD_CARD_CHIP_SELECT = 10;

class Led {
    public:
        void begin(){
            pinMode(LED_BUILTIN,OUTPUT);
            digitalWrite(LED_BUILTIN,LOW);
        }

        void flash(){
            digitalWrite(LED_BUILTIN,HIGH);
            delay(25);
            digitalWrite(LED_BUILTIN,LOW);
        }
};
Led led;

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

        uint32_t get_time(){
            uint32_t elapsedTime = (millis()-startTime)/1000; // in seconds

            seconds = elapsedTime % 60;
            minutes = (elapsedTime % 3600) / 60;
            hours = elapsedTime / 3600;

            if (minutes<10){
                hours*=10;
            }
            else if (seconds<10){
                minutes*=10;
            }
            return hours*10000 + minutes*100 + seconds;
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
        bool broadcasting = false;
        uint32_t broadcastStartTime = 0;

        int ID = 12;
        int packetCount = 0;

        int frequency = 866E6;
        int commsBaudRate = 9600;
        float percentActive = 0.1;
        int sleepAmount;

        static const int COMPONENT_COUNT = 4;

        std::string printBuffer = "";
        unsigned long bitSize;

        const double SEA_LEVEL_PRESSURE_HPA = (1013.25); // FIXME: replace by value true locally

        void prints(float data, char separator = ','){
            COMMS_SERIAL.print(data);
            COMMS_SERIAL.print(separator);
            printBuffer += std::to_string(data);
            bitSize += sizeof(data) + sizeof(separator);
        }

        void prints(int data, char separator = ','){
            COMMS_SERIAL.print(data);
            COMMS_SERIAL.print(separator);
            printBuffer += std::to_string(data);
            bitSize += sizeof(data) + sizeof(separator);
        }

        void prints(uint32_t data, char separator = ','){
            COMMS_SERIAL.print(data);
            COMMS_SERIAL.print(separator);
            printBuffer += std::to_string(data);
            bitSize += sizeof(data) + sizeof(separator);
        }

        void prints(char data, char separator = ','){
            COMMS_SERIAL.print(data);
            COMMS_SERIAL.print(separator);
            printBuffer += std::to_string(data) + separator;
            bitSize += sizeof(data) + sizeof(separator);
        }

    public:
    /*
        String parse(){
            String packet = "";
            packet += this->format_data(ID) + ",";
            packet += this->format_data(packetCount);
            packet += timer.get_time() + ",";
            
            packet += this->format_data(bme.readTemperature());
            packet += this->format_data(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA));
            packet += this->format_data(bme.readHumidity()); // TODO: add readings for all components
            
            packetCount=packetCount+1;
            return packet + this->get_checksum(packet);
        }
    */
        void start_broadcast(){
            broadcasting = true;
        }

        void broadcast(){
            /*
            uint32_t currentTime = millis();
            uint32_t startTime = broadcastStartTime;
            if (broadcasting && (startTime+sleepAmount<currentTime)){
                COMMS_SERIAL.println(currentTime);COMMS_SERIAL.println(startTime);COMMS_SERIAL.println(sleepAmount);
                broadcastStartTime = currentTime;
                this->telemetry_send();
                //set_sleep_amount();
                led.flash();
            }
            */
            if (broadcasting && (millis()-broadcastStartTime>=1000)){
                broadcastStartTime = millis();
                this->telemetry_send();
                //set_sleep_amount();
            }

        }

        void telemetry_send(){
            int dataIndex = 0;
            bitSize = 0;
            printBuffer = "";
            
            this->prints(ID);
            this->prints(packetCount++);
            this->prints(timer.get_time());

            // BME280
            if (bme.begin()){
                this->prints(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA));
                this->prints(bme.readTemperature());
                this->prints(bme.readHumidity());
            }else{
                this->prints(0);
                this->prints(0);
                this->prints(0);
            }

            // CHECKSUM
            COMMS_SERIAL.print('*');
            byte checksum = this->get_checksum(printBuffer, dataIndex);
            COMMS_SERIAL.println(checksum);
            bitSize += sizeof('*') + sizeof(checksum);
            bitSize *= 8;

            //this->write(printBuffer, dataIndex);
        }

        void begin(){
            COMMS_SERIAL.begin(commsBaudRate);
            while (!COMMS_SERIAL){
            }
            sleepAmount = 1000;
            packetCount = 0;
            timer.reset();
            this->start_broadcast();
            this->send("Telemetry INIT: OK");
        }

        void set_sleep_amount(){
            sleepAmount = 1000 - (bitSize / commsBaudRate)*1000;
        }

        byte get_checksum(std::string data, int size) {
            byte checksum = 0;
            for (char c : data){
                checksum ^= c;
            }
            return checksum;
        }

        void send(const String& message){
            COMMS_SERIAL.println(message);            
        }

        void write(std::string data, int valueCount){
            if (SD.begin(SD_CARD_CHIP_SELECT)){
                File logFile = SD.open("logFile.txt", FILE_WRITE);
                if (logFile){
                    for (int i = 0; i<valueCount; ){
                        logFile.print(printBuffer[i]);
                        logFile.print(',');
                    }
                    logFile.println();
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
};



Telemetry data;

void setup(){
    // TODO: add component validation
    data.sd_validate();
    data.begin();
    led.begin();
    led.flash();
}

void loop(){
    data.broadcast();

}