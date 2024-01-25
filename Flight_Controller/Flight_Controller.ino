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
#define COMMS_SERIAL    Serial
// GPS - serial through pin(8,7) [TX,RX]
#define GPS_SERIAL      Serial2


int guvaAnalogPin = 14;
float guvaOperatingVoltage = 3.3;
int resolution = 1024;

float guvaCalibrationValue = 0;


float read_guva_intensity(){
    float sensorValue = analogRead(guvaAnalogPin);
    return (sensorValue/resolution*guvaOperatingVoltage)-guvaCalibrationValue;
}

void calibrate(){
    guvaCalibrationValue = read_guva_intensity();
}

TinyGPSPlus gps;
int gpsBaud = 9600;

const char *gpsStream =
    "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
    "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
    "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
    "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
    "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
    "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";


void gps_begin(){
    GPS_SERIAL.begin(gpsBaud);
}

void feed_gps(){
    if (GPS_SERIAL.available()){
        gps.encode(GPS_SERIAL.read());
    }
    /*
    if (*gpsStream){
        encode(*gpsStream++);
    }
    */
}


void led_begin(){
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
}

void flash(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(25);
    digitalWrite(LED_BUILTIN,LOW);
}



unsigned long missionStartTime;

unsigned long processStart;

void mission_begin(){
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


int radioFrequency = 868000000;
int syncWord;
int bandwidth;

bool is_comms_available(){
    return COMMS_SERIAL;
}

void send_request(String command){
    COMMS_SERIAL.println(command);
}

void receive(int milli){
    COMMS_SERIAL.println("radio rx " + milli);
}

uint32_t broadcastStartTime = 0;
bool inFlight = false;

int packetCount = 0;
const char separator = ',';
const char checksumIdentifier = '*';

const String telemetryPreamble = "radio tx ";
const String csvHeader = "packet count,mission time,internal temperature,barometric altitude,external temperature (bme280),humidity,gps age,latitude,longitude,gps altitude,acceleration (x),acceleration (y),acceleration (z),inclination (x),inclination (y),inclination (z),external temperature (mpu6050),light intenity,sleep amount,checksum";
const String* headerArray = string_to_array(csvHeader);
const int indexPacketCount = 0;
const int indexMissionTime = 1;
const int indexInternalTemperature = 2;
const int indexBarometricAltitude = 3;
const int indexExternalBmeTemperature = 4;
const int indexHumidity = 5;
const int indexGpsAge = 6;
const int indexLatitude = 7;
const int indexLongitude = 8;
const int indexGpsAltitude = 9;
const int indexAccelerationX = 10;
const int indexAccelerationY = 11;
const int indexAccelerationZ = 12;
const int indexGyroscopeX = 13;
const int indexGyroscopeY = 14;
const int indexGyroscopeZ = 15;
const int indexExternalMpuTemperature = 16;
const int indexLightIntensity = 17;
const int indexSleepAmount = 18;
const int indexChecksum = 19;

int commsBaudRate = 115200;
float percentActive = 10;
float sleepAmount;

String printBuffer = "";
float transmissionSize;

Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

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


void handle_data(){
    while (1){
        uint32_t elapsedTime = millis()-broadcastStartTime;
        if (inFlight) {
            if (elapsedTime>=sleepAmount){
                broadcastStartTime = millis();
                telemetry_send();
                set_sleep_amount();
                flash();
            }
            feed_gps();
        } else {
            delegate_telemetry();
        }
        threads.yield();
    }
}

void telemetry_send(){
    while (!(COMMS_SERIAL.availableForWrite()>0)){
        delay(1);
    } // FIXME: high risk loop 
    transmissionSize = 0;
    printBuffer = "";

    prints(String(packetCount++));                                          // packet count
    prints(String(get_time()));                                             // current mission time

    // SYSTEM
    prints(tempmonGetTemp());                                               // internal temperature

    // GPS - WLR089u0
    if (gps.location.isValid()){
        prints(String(gps.location.age(

        )));                                 // gps age
        prints(String(gps.location.lat()));                                 // latitude
        prints(String(gps.location.lng()));                                 // longitude
    }else{
        for (int i = 0; i<3; i++){
            prints("#");
        }
    }

    if (gps.altitude.isValid()){
        prints(String(gps.altitude.meters()));                              // gps altitude
    }else{
        prints("#");
    }

    // BME280
    if (bme.begin(0x76)){
        prints(String(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA)));           // altitude
        prints(String(bme.readTemperature()));                              // temperature
        prints(String(bme.readHumidity()));                                 // humidity
    }else{
        for (int i = 0; i<3; i++){
            prints("#");
        }
    }

    // MPU-6050
    if (mpu.begin(0x68)){
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        // acceleration
        prints(String(a.acceleration.x));                                   // acceleration x
        prints(String(a.acceleration.y));                                   // acceleration y
        prints(String(a.acceleration.z));                                   // acceleration z
        // gyroscope
        prints(String(g.gyro.x));                                           // gyro x
        prints(String(g.gyro.y));                                           // gyro y
        prints(String(g.gyro.z));                                           // gyro z
        // temperature
        prints(String(temp.temperature));                                   // mpu ext. temperature
    } else {
        for (int i = 0; i<7; i++){
            prints("#");
        }
    }

    // GUVA-S12SD
    prints(String(read_guva_intensity()));                                  // light intensity

    prints(String(sleepAmount));                                            // sleep time

    // CHECKSUM
    String checksum = get_checksum(printBuffer);                            // checksum
    printBuffer += checksumIdentifier+checksum;
    transmissionSize += sizeof(printBuffer);
    transmissionSize *= 8;
    COMMS_SERIAL.println("radio tx " + string_to_hex(printBuffer) + " 1");
    Serial.println(printBuffer);
}

void delegate_telemetry(){
    if (COMMS_SERIAL.available()){
        flash();
        String incoming = COMMS_SERIAL.readString();
        
        if (incoming.startsWith(telemetryPreamble)){
            incoming = incoming.substring(telemetryPreamble.length());
            incoming = hex_to_string(incoming);

            // TODO: write incoming data to sd
            if (checksum_invalid(incoming)){
                // TODO: write error code to indicate deviation from checksum
            }
            
            display_incoming_data(incoming);
        }
    }
}

void telemetry_begin(){
    COMMS_SERIAL.begin(commsBaudRate);
    Serial.begin(9600);
    sleepAmount = 1000;
    packetCount = 0;
    mission_begin();
    // TODO: write csv header to sd card
}

void display_incoming_data(String data){
}

String* string_to_array(String data){
    int dataCount = 1;
    for (int i = 0; i < data.length(); i++){
        if (data[i] == separator){
            dataCount++;
        }
    }

    String* resultArray = new String[dataCount];

    // Use strtok to split the input string at commas
    char* token = strtok(const_cast<char*>(data.c_str()), ",");
    int index = 0;

    // Loop through the tokens and store them in the array
    while (token != NULL) {
    resultArray[index++] = String(token);
    token = strtok(NULL, ",");
    }

    return resultArray;
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
    if (get_checksum(data)==checksum){
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


void setup(){
    // TODO: add component validation
    telemetry_begin();
    led_begin();
    flash();
    threads.addThread(handle_data);
}

void loop(){
}