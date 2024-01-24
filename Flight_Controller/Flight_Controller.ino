#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <CRC32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <TeensyThreads.h>

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

class MyGPS : public TinyGPSPlus {
    private:
        int gpsBaud;

        const char *gpsStream =
            "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
            "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
            "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
            "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
            "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
            "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

    public:
        void begin(){
            this->set_baud(9600);
            GPS_SERIAL.begin(gpsBaud);
        }

        void set_baud(int baud){
            gpsBaud = baud;
        }

        void feed_gps(){
            if (GPS_SERIAL.available()){
                this->encode(GPS_SERIAL.read());
            }
            /*
            if (*gpsStream){
                this->encode(*gpsStream++);
            }
            */
        }
};
MyGPS gps;

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
            uint32_t elapsedTime = (millis()-missionStartTime) / 10; // in deciseconds xD
            return float(elapsedTime)/100; // should return seconds to one decimal precision
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
        uint32_t broadcastStartTime = 0;
        bool inFlight = true;

        int packetCount = 0;
        const char separator = ',';
        const char checksumIdentifier = '*';

        const String telemetryPreamble = "radio tx ";

        int commsBaudRate = 115200;
        float percentActive = 10;
        float sleepAmount;

        String printBuffer = "";
        float transmissionSize;

        Adafruit_BME280 bme;
        Adafruit_MPU6050 mpu;
        LightSensor guva = LightSensor(14,3.3);

        const double SEA_LEVEL_PRESSURE_HPA = (1013.25); // FIXME: replace by value true locally

        void prints(String data){
            String dataBlock = data + separator;
            printBuffer += dataBlock;
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

        String hex_to_string(String hexString){
            String asciiString = "";

            for (unsigned int i = 0; i < hexString.length(); i += 2) {
                String hexPair = hexString.substring(i, i + 2);
                int intValue = strtol(hexPair.c_str(), NULL, 16);
                char asciiChar = char(intValue);
                asciiString += asciiChar;
            }

            return asciiString;
        }

    public:
        Telemetry(){
            this->begin();
        }

        void handle_data(){ 
            uint32_t elapsedTime = millis()-broadcastStartTime;
            if (inFlight) {
                if (elapsedTime>=sleepAmount){
                    broadcastStartTime = millis();
                    this->telemetry_send();
                    set_sleep_amount();
                    led.flash();
                }
                gps.feed_gps();
            } else {
                delegate_telemetry();
            }
        }

        void telemetry_send(){
            while (!(COMMS_SERIAL.availableForWrite()>0)){
                delay(1);
            } // FIXME: high risk loop 
            transmissionSize = 0;
            printBuffer = "";

            this->prints(String(packetCount++));                                        // packet count
            this->prints(String(timer.get_time()));                                     // current mission time

            // SYSTEM
            this->prints(tempmonGetTemp());                                             // internal temperature

            // GPS - WLR089u0
            if (gps.location.isValid()){
                this->prints(String(gps.location.age()));
                this->prints(String(gps.location.lat()));
                this->prints(String(gps.location.lng()));
            }else{
                for (int i = 0; i<3; i++){
                    this->prints("#");
                }
            }

            if (gps.altitude.isValid()){
                this->prints(String(gps.altitude.meters()));
            }else{
                this->prints("#");
            }

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

            this->prints(String(sleepAmount));                                          // sleep time

            // CHECKSUM
            String checksum = this->get_checksum(printBuffer);                          // checksum
            printBuffer += checksumIdentifier+checksum;
            transmissionSize += sizeof(printBuffer);
            transmissionSize *= 8;
            COMMS_SERIAL.println("radio tx " + this->string_to_hex(printBuffer) + " 1");
            Serial.println(printBuffer);
        }

        void delegate_telemetry(){
            if (COMMS_SERIAL.available()){
                String incoming = COMMS_SERIAL.readString();
                
                if (incoming.startsWith(telemetryPreamble)){
                    incoming = incoming.substring(telemetryPreamble.length());
                    incoming = this->hex_to_string(incoming);

                    // TODO: write incoming data to sd
                    if (this->checksum_invalid(incoming)){
                        // TODO: write error code to indicate deviation from checksum
                    }
                }
            }
        }

        void begin(){
            COMMS_SERIAL.begin(commsBaudRate);
            Serial.begin(9600);
            sleepAmount = 1000;
            packetCount = 0;
            mpu.begin(0x68);
            mpu.reset();
            timer.begin();
            // TODO: write csv header to sd card
        }

        void set_sleep_amount(){ // FIXME: gives negative sleep values
            uint16_t transmissionTime = (transmissionSize / commsBaudRate) * 1000;
            sleepAmount = (transmissionTime/percentActive)*(100-percentActive);
        }

        String get_checksum(String data){
            unsigned int checksum = CRC32::calculate(data.c_str(), data.length());
            char checksumStr[3];
            snprintf(checksumStr, sizeof(checksumStr), "%02X", checksum);
            return String(checksumStr);
        }

        bool checksum_invalid(String data){
            String checksum = data.substring(data.indexOf(checksumIdentifier));
            if (this->get_checksum(data)==checksum){
                return false;
            } else {
                return true;
            }
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
    data.handle_data();
}