#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <CRC32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>

#include <core_pins.h>
#include <usb_serial.h>
#include <math.h>
#include <string>

// SERIAL COMMS  
// COMMS - **Serial1** through pin(1,0) [TX,RX]      // USB SERIAL - **Serial** through USB port
#define COMMS_SERIAL    Serial  // FIXME: change to Serial1 on final version
// GPS - serial through pin(8,7) [TX,RX]
#define GPS_SERIAL      Serial2




// GLOBALS


class LightSensor {
    private:
        int analogPin;
        float operatingVoltage;
        int resolution = 1024;

        float calibrationValue = 0;
        
    public:
        LightSensor(int pin, float voltage) : analogPin(pin), operatingVoltage(voltage){}

        float ReadIntensity(){
            float sensorValue = analogRead(analogPin);
            return (sensorValue/resolution*operatingVoltage)-calibrationValue;
        }

        void Calibrate(){
            calibrationValue = this->ReadIntensity();
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
            this->SetBaud(9600);
            GPS_SERIAL.begin(gpsBaud);
        }

        void SetBaud(int baud){
            gpsBaud = baud;
        }

        void FeedGPS(){
            /*
            while(GPS_SERIAL.available()){
                this->encode(GPS_SERIAL.read());
            }
            */
           while (*gpsStream){
            this->encode(*gpsStream++);
           }
        }
};
MyGPS gps;

class Led {
    public:
        void begin(){
            pinMode(LED_BUILTIN,OUTPUT);
            digitalWrite(LED_BUILTIN,LOW);
        }

        void Flash(){
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
            this->Reset();
        }

        void Reset(){
            missionStartTime = millis();
            processStart = 0;
        }
        
        String GetTimestamp(){
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
        

        float GetTime(){
            uint32_t elapsedTime = (millis()-missionStartTime) / 100; // in deciseconds xD
            return float(elapsedTime)/10; // should return seconds to one decimal precision
        }

        void TrueSleep(int milli){
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
        uint32_t sleepAmount;

        // **COMPONENTS**
        static const int COMPONENT_COUNT = 4;
        Adafruit_BME280 bme;
        Adafruit_MPU6050 mpu;
        LightSensor guva = LightSensor(14,3.3);

        std::string printBuffer = "";
        uint32_t transmissionSize;

        const double SEA_LEVEL_PRESSURE_HPA = (1013.25); // FIXME: replace by value true locally

        void prints(float data, char separator = ','){
            if (COMMS_SERIAL){
                COMMS_SERIAL.print(data);
                COMMS_SERIAL.print(separator);
            }
            printBuffer += std::to_string(data) + separator;
            transmissionSize += sizeof(data) + sizeof(separator);
        }

        void prints(int data, char separator = ','){
            if (COMMS_SERIAL){
                COMMS_SERIAL.print(data);
                COMMS_SERIAL.print(separator);
            }
            printBuffer += std::to_string(data) + separator;
            transmissionSize += sizeof(data) + sizeof(separator);
        }

        void prints(uint32_t data, char separator = ','){
            if (COMMS_SERIAL){
                COMMS_SERIAL.print(data);
                COMMS_SERIAL.print(separator);
            }
            printBuffer += std::to_string(data) + separator;
            transmissionSize += sizeof(data) + sizeof(separator);
        }

        void prints(char data, char separator = ','){
            if (COMMS_SERIAL){
                COMMS_SERIAL.print(data);
                COMMS_SERIAL.print(separator);
            }
            printBuffer += std::to_string(data) + separator;
            transmissionSize += sizeof(data) + sizeof(separator);
        }

        void prints(String data, char separator = ','){
            if (COMMS_SERIAL){
                COMMS_SERIAL.print(data);
                COMMS_SERIAL.print(separator);
            }
            printBuffer += data.c_str() + separator;
            transmissionSize += sizeof(data) + sizeof(separator);
        }

    public:
        Telemetry(){
            this->begin();
        }

        void StartBroadcast(){
            broadcasting = true;
        }

        void Broadcast(){
            uint32_t elapsedTime = millis()-broadcastStartTime;
            if (broadcasting && (elapsedTime>=sleepAmount)){
                broadcastStartTime = millis();
                this->SendTelemetry();
                SetSleepAmount(elapsedTime);
                led.Flash();
            }
            gps.FeedGPS();
        }

        void SendTelemetry(){
            transmissionSize = 0;
            printBuffer = "";
            
            this->prints(ID);
            this->prints(packetCount++);
            this->prints(timer.GetTime());

            // GPS - WLR089u0
            if (gps.location.isValid()){
                this->prints(gps.location.age()); // TODO: decide if age is needed or not
                this->prints(float(gps.location.lat()));
                this->prints(float(gps.location.lng()));
            }else{
                for (int i = 0; i<3; i++){
                    this->prints(0);
                }
            }

            if (gps.altitude.isValid()){
                this->prints(float(gps.altitude.meters()));
            }else{
                this->prints(0);
            }

            // BME280
            if (bme.begin(0x76)){
                this->prints(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA));
                this->prints(bme.readTemperature());
                this->prints(bme.readHumidity());
            }else{
                for (int i = 0; i<3; i++){
                    this->prints(0);
                }
            }

            // MPU-6050
            if (mpu.begin(0x68)){
                sensors_event_t a, g, temp;
                mpu.getEvent(&a, &g, &temp);
                // acceleration
                this->prints(a.acceleration.x);
                this->prints(a.acceleration.y);
                this->prints(a.acceleration.z);
                // gyroscope
                this->prints(g.gyro.x);
                this->prints(g.gyro.y);
                this->prints(g.gyro.z);
                // temperature
                this->prints(temp.temperature);
            }else{
                for (int i = 0; i<7; i++){
                    this->prints(0);
                }
            }

            // GUVA-S12SD
            this->prints(guva.ReadIntensity());

            // SYSTEM
            this->prints(tempmonGetTemp()); // internal temperature
            // TRANSMISSION SIZE in Bytes (without checksum)
            this->prints(transmissionSize);
            // SLEEP AMOUNT
            this->prints(sleepAmount);

            // CHECKSUM
            COMMS_SERIAL.print('*');
            String checksum = this->GetChecksum(printBuffer);
            COMMS_SERIAL.println(checksum);
            transmissionSize += sizeof('*') + sizeof(checksum);
            transmissionSize *= 8;
        }

        void begin(){
            COMMS_SERIAL.begin(commsBaudRate);
            sleepAmount = 1000;
            packetCount = 0;
            timer.Reset();
            this->StartBroadcast();
        }

        void SetSleepAmount(uint32_t elapsedTime){
            uint32_t transmissionTime = (transmissionSize / commsBaudRate) * 1000;
            elapsedTime -= transmissionTime;
            sleepAmount = (1-percentActive)*10 * transmissionTime - elapsedTime;
        }

        String GetChecksum(std::string data){
            unsigned int checksum = CRC32::calculate(data.c_str(), data.length());
            char checksumStr[3];
            snprintf(checksumStr, sizeof(checksumStr), "%02X", checksum);
            return String(checksumStr);
        }

        void send(const String& message){
            COMMS_SERIAL.println(message);            
        }
};



Telemetry data;

void setup(){
    // TODO: add component validation
    data.begin();
    led.begin();
    led.Flash();
}

void loop(){
    data.Broadcast();
}