#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <CRC32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>

#include <core_pins.h>
#include <usb_serial.h>
#include <math.h>
#include <string>

// SERIAL COMMS  
// LORA RADIO - Serial1      // USB SERIAL - **Serial** through USB port
#define COMMS_SERIAL    Serial1
// GPS - serial through pin(8,7) [TX,RX]
#define GPS_SERIAL      Serial2

class LightSensor {
    private:
        int analogPin;
        float operatingVoltage;
        int resolution = 1024;

        float calibrationValue = 0;
        
    public:
        LightSensor(int pin, float voltage) : analogPin(pin), operatingVoltage(voltage){}

        float readIntensity(){
            float sensorValue = analogRead(analogPin);
            return (sensorValue/resolution*operatingVoltage)-calibrationValue;
        }

        void calibrate(){
            calibrationValue = this->readIntensity();
        }
};

// **COMPONENTS**


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
        unsigned long missionStartTime;

        unsigned long processStart;
    public:
        MissionTime(){
            this->begin();
        }

        void begin(){
            missionStartTime = millis();
            processStart = 0;
        }
        
        String get_timestamp(){
            unsigned int seconds, minutes, hours;
            uint32_t elapsedTime = (millis()-missionStartTime)/1000; // in seconds

            seconds = elapsedTime % 60;
            minutes = (elapsedTime % 3600) / 60;
            hours = elapsedTime / 3600;
            
            String timestamp = "";
            timestamp += hours + ":";
            timestamp += minutes + ":";
            timestamp += seconds;

            return timestamp;
        }
        

        float get_time(){
            uint32_t elapsedTime = (millis()-missionStartTime) / 100; // in deciseconds xD
            return float(elapsedTime)/10; // should return seconds to one decimal precision
        }

        void true_sleep(int milli){
            delay(milli - (millis()-processStart));
            processStart = millis();
        }
};

class Radio{
    private:
        int frequency = 868000000;
        int syncWord;
        int bandwidth;

    public:
        bool is_available(){
            return COMMS_SERIAL;
        }

        void send_request(String command){
            COMMS_SERIAL.println(command);
        }

        void receive(int milli){
            COMMS_SERIAL.println("radio rx " + milli);
        }

        
};

class Telemetry{
    private:
        Radio lora;
        MissionTime timer;
        bool broadcasting = false;
        uint32_t broadcastStartTime = 0;

        int packetCount = 0;
        char separator = ',';
        char checksumIdentifier = '*';

        int commsBaudRate = 115200;
        float percentActive = 0.1; // decimal notation
        float sleepAmount;

        String printBuffer = "";
        float transmissionSize;

        Adafruit_BME280 bme;
        Adafruit_MPU6050 mpu;
        LightSensor guva = LightSensor(14,3.3);

        const double SEA_LEVEL_PRESSURE_HPA = (1013.25); // FIXME: replace by value true locally

        void prints(String data){
            /*
            if (this->is_comms_available()){
                COMMS_SERIAL.print(data);
                COMMS_SERIAL.print(separator);
            }
            */
            String dataBlock = this->string_to_hex(data + separator);
            printBuffer += dataBlock;
            transmissionSize += sizeof(dataBlock);
        }

        String string_to_hex(String data){
            String hexString = "";
  
            for (int i = 0; i < data.length(); i++) {
                char currentChar = data.charAt(i);
                
                // Convert the character to its hexadecimal representation
                char hexChars[3];
                sprintf(hexChars, "%02X", currentChar);
                
                // Append the hexadecimal representation to the result string
                hexString += hexChars;
            }
            
            return hexString;
        }

    public:
        Telemetry(){
            this->begin();
        }

        void start_broadcast(){
            broadcasting = true;
        }

        void broadcast(){
            uint32_t elapsedTime = millis()-broadcastStartTime;
            if (broadcasting){// && (elapsedTime>=sleepAmount)){
                broadcastStartTime = millis();
                //COMMS_SERIAL.print("radio tx ");
                this->telemetry_send();
                //COMMS_SERIAL.println(" 1");
                set_sleep_amount(elapsedTime);
                led.flash();
                delay(1000);
            }
        }

        void telemetry_send(){
            while (!(COMMS_SERIAL.availableForWrite()>0)); // FIXME: high risk loop 
            transmissionSize = 0;
            printBuffer = "";

            this->prints(String(packetCount++));                                        // packet count
            this->prints(String(timer.get_time()));                                     // current mission time

            // SYSTEM
            this->prints(tempmonGetTemp());                                     // internal temperature

            // BME280
            if (bme.begin(0x76)){
                this->prints(String(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA)));         // altitude
                this->prints(String(bme.readTemperature()));                            // temperature
                this->prints(String(bme.readHumidity()));                               // humidity
            }else{
                for (int i = 0; i<3; i++){
                    this->prints("#");
                }
            }

            // MPU-6050
            if (mpu.begin(0x68)){
                sensors_event_t a, g, temp;
                mpu.getEvent(&a, &g, &temp);
                // acceleration
                this->prints(String(a.acceleration.x));                                 // acceleration x
                this->prints(String(a.acceleration.y));                                 // acceleration y
                this->prints(String(a.acceleration.z));                                 // acceleration z
                // gyroscope
                this->prints(String(g.gyro.x));                                         // gyro x
                this->prints(String(g.gyro.y));                                         // gyro y
                this->prints(String(g.gyro.z));                                         // gyro z
                // temperature
                this->prints(String(temp.temperature));                                 // mpu ext. temperature
            }else{
                for (int i = 0; i<7; i++){
                    this->prints("#");
                }
            }

            // GUVA-S12SD
            this->prints(String(guva.readIntensity()));                                 // light intensity

            this->prints(String(transmissionSize));                                     // tx size (w/o checksum)
            this->prints(String(sleepAmount));                                          // sleep time

            // CHECKSUM
            String checksum = this->get_checksum(printBuffer);                  // checksum
            printBuffer += this->string_to_hex(checksumIdentifier+checksum);
            transmissionSize += sizeof(checksumIdentifier + checksum);
            transmissionSize *= 8;
            COMMS_SERIAL.println("radio tx " + printBuffer + " 1");
            Serial.println(printBuffer);
        }

        void begin(){
            COMMS_SERIAL.begin(commsBaudRate);
            while (!COMMS_SERIAL);
            
            sleepAmount = 1000;
            packetCount = 0;
            timer.begin();
            this->start_broadcast();
        }

        void set_sleep_amount(uint32_t elapsedTime){
            uint32_t transmissionTime = (transmissionSize / commsBaudRate) * 1000;
            elapsedTime -= transmissionTime;
            sleepAmount = (1-percentActive)*10 * transmissionTime - elapsedTime;
        }

        String get_checksum(String data){
            unsigned int checksum = CRC32::calculate(data.c_str(), data.length());
            char checksumStr[3];
            snprintf(checksumStr, sizeof(checksumStr), "%02X", checksum);
            return String(checksumStr);
        }

        void send(const String& message){
            COMMS_SERIAL.println(message);            
        }

        bool is_lora(){
            return false;
        }

        // .availableForWrite() is shared between Serial and LoRaClass
        bool is_comms_available(){
            return COMMS_SERIAL;
        }
};

Telemetry data;

void setup(){
    // TODO: add component validation
    data.begin();
    led.begin();
    led.flash();
}

void loop(){
    data.broadcast();
}