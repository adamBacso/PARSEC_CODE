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

        float get_time(){
            unsigned long elapsedTime = (millis()-startTime)/1000; // in seconds

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
        uint32_t broadcastStartTime;

        float ID = 12;
        float packetCount = 0;

        int frequency = 866E6;
        int commsBaudRate = 9600;
        float percentActive = 0.1;
        int sleepAmount;

        static const int COMPONENT_COUNT = 4;

        float printBuffer[COMPONENT_COUNT+3] = {0};
        unsigned long bitSize;

        const double SEA_LEVEL_PRESSURE_HPA = (1013.25); // FIXME: replace by value true locally

        void prints(float data, int index, char separator = ','){
            if (index < COMPONENT_COUNT+3){
                COMMS_SERIAL.print(data);
                COMMS_SERIAL.print(separator);
                this-> printBuffer[index] = data;
                bitSize += sizeof(data)*8 + sizeof(separator)*8;
            }
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
            uint32_t currentTime = millis();
            uint32_t startTime = broadcastStartTime;
            if (broadcasting && (startTime+sleepAmount-10<currentTime&&startTime+sleepAmount+10>currentTime)){
                broadcastStartTime = currentTime;
                this->telemetry_send();
                set_sleep_amount();
            }
        }

        void telemetry_send(){
            int dataIndex = 0;
            bitSize = 0;
            
            this->prints(ID, dataIndex++);
            this->prints(packetCount++, dataIndex++);
            this->prints(timer.get_time(), dataIndex++);

            // BME280
            if (bme.begin()){
                this->prints(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA), dataIndex++);
                this->prints(bme.readTemperature(), dataIndex++);
                this->prints(bme.readHumidity(), dataIndex++);
            }else{
                this->prints(0, dataIndex++);
                this->prints(0, dataIndex++);
                this->prints(0, dataIndex++);
            }

            // CHECKSUM
            //activeSerial.print('*'); activeSerial.println(get_checksum(printBuffer, valueCount));
            COMMS_SERIAL.print('*');
            byte checksum = this->get_checksum(printBuffer, dataIndex);
            COMMS_SERIAL.println(checksum);
            bitSize += sizeof('*')*8 + sizeof(checksum)*8;

            this->write(printBuffer, dataIndex);
        }

        void begin(){
            COMMS_SERIAL.begin(commsBaudRate);
            while (!COMMS_SERIAL){
            }
            sleepAmount = 0;
            packetCount = 0;
            timer.reset();
            this->start_broadcast();
            this->send("Telemetry INIT: OK");
        }

        void set_sleep_amount(){
            sleepAmount = (1-percentActive) * (bitSize / commsBaudRate);
        }

        byte get_checksum(float data[], int size) {
            byte checksum = 0;
            for (int i = 0; i < size; i++) {
                byte* bytes = reinterpret_cast<byte*>(&data[i]);
                for (unsigned int j = 0; j < sizeof(float); j++) {
                    checksum ^= bytes[j];
                }
            }
            return checksum;
        }

        void send(const String& message){
            COMMS_SERIAL.println(message);            
        }

        void write(float data[], int valueCount){
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
Led led;


void setup(){
    // TODO: add component validation
    data.sd_validate();
    data.begin();
    led.begin();
    led.flash();
}

void loop(){
    data.broadcast();
    led.flash();
}