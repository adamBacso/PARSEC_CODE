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
#define COMMS_SERIAL    Serial  
// GPS - serial through pin(8,7) [TX,RX]
#define GPS_SERIAL      Serial2



// BME280
// https://randomnerdtutorials.com/bme280-sensor-arduino-pressure-temperature-humidity/
Adafruit_BME280 bme;
// GLOBALS
const int SD_CARD_CHIP_SELECT = 10;
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

        float ID = 12;
        float packetCount = 0;

        int frequency;
        int commsBaudRate;
        int percentActive;

        static const int COMPONENT_COUNT = 4;

        float printBuffer[COMPONENT_COUNT+3] = {0};

        const double SEA_LEVEL_PRESSURE_HPA = (1013.25); // FIXME: replace by value true locally

        void prints(float data, int index, char separator = ','){
            COMMS_SERIAL.print(data);
            COMMS_SERIAL.print(separator);

            if (index < COMPONENT_COUNT+3){
               this-> printBuffer[index] = data;
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
        void startBroadcast(){
            broadcasting = true;
            while (broadcasting){
                this->serial_send(); 
                timer.true_sleep(1000); // FIXME: replace by threadless sleep; FIXME: determine sleep amount from percentActive and commsBaudRate
            }
        }

        void serial_send(){
            int dataIndex = 0;
            // HEADER
            this->prints(ID, dataIndex++);
            this->prints(packetCount++, dataIndex++);
            this->prints(timer.get_time(), dataIndex++);

            // BME280
            if (bme.begin()){
                this->prints(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA), dataIndex++);
                this->prints(bme.readTemperature(), dataIndex++);
                this->prints(bme.readHumidity(), dataIndex++);
            }

            // CHECKSUM
            //activeSerial.print('*'); activeSerial.println(get_checksum(printBuffer, valueCount));
            COMMS_SERIAL.print('*');
            COMMS_SERIAL.println(this->get_checksum(printBuffer, dataIndex));

            this->write(printBuffer, dataIndex);
        }

        void init(){
            packetCount = 0;
            timer.reset();
            this->send("Telemetry INIT: OK");
        }

        byte get_checksum(float data[], int size) {
            byte checksum = 0;
            for (int i = 0; i < size; i++) {
                byte* bytes = reinterpret_cast<byte*>(&data[i]);
                for (int j = 0; j < sizeof(float); j++) {
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
    
    COMMS_SERIAL.begin(9600);
    // TODO: add component validation
    data.sd_validate();
    data.init();
    led.init();
    mode = 1;
    led.flash();
}

void loop(){
    led.flash();
}