#include <PWMServo.h>
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
// LORA RADIO // USB SERIAL - **Serial** through USB port
#define COMMS_SERIAL    Serial2

int commsBaudRate = 115200;
// GPS
#define GPS_SERIAL      Serial1

// PINS

int resolution = 1024;

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


void led_begin(void){
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
}

void flash(bool isThreaded = false){
    digitalWrite(LED_BUILTIN,HIGH);
    if (isThreaded){
        threads.delay(25);
    } else {
        delay(25);
    }
    digitalWrite(LED_BUILTIN,LOW);
}



unsigned long missionStartTime;

unsigned long processStart;

void mission_begin(void){
    missionStartTime = millis();
    processStart = 0;
}

String get_timestamp(void){
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


float get_time(void){
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

bool is_comms_available(void){
    return COMMS_SERIAL;
}

void send_request(String command){
    COMMS_SERIAL.println(command);
}

void receive(void){
    COMMS_SERIAL.println("radio rx 0");
}

void stop_reception(void){
    COMMS_SERIAL.println("radio rxstop");
}

uint32_t broadcastStartTime = 0;
bool inFlight = true;

int packetCount = 0;
const char separator = ',';
const char checksumIdentifier = '*';

const String telemetryPreamble = "radio_rx ";
const String commandPreamble = "CMD";
const String csvHeader = "packet count,mission time,internal temperature,barometric altitude,external temperature (bme280),humidity,gps age,latitude,longitude,gps altitude,acceleration (x),acceleration (y),acceleration (z),inclination (x),inclination (y),inclination (z),external temperature (mpu6050),,uptime,sleep amount,checksum";
const String* headerArray = string_to_array(csvHeader);
/*
int index = 0;
int indexPacketCount = index++;
int indexMissionTime = index++;
int indexInternalTemperature = index++;
int indexBarometricAltitude = index++;
int indexExternalBmeTemperature = index++;
int indexHumidity = index++;
int indexGpsAge = index++;
int indexLatitude = index++;
int indexLongitude = index++;
int indexGpsAltitude = index++;
int indexAccelerationX = index++;
int indexAccelerationY = index++;
int indexAccelerationZ = index++;
int indexGyroscopeX = index++;
int indexGyroscopeY = index++;
int indexGyroscopeZ = index++;
int indexExternalMpuTemperature = index++;
int indexUptime = index++;
int indexSleepAmount = index++;
int indexChecksum = index++;
*/

float percentActive = 10;
float sleepAmount;

String printBuffer = "";
float transmissionSize;

Adafruit_BME280 bme; int bmeAddress = 0x76;
Adafruit_MPU6050 mpu; int mpuAddress = 0x68;

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
uint32_t elapsedTime;
void handle_data(void){
    flash();
    while (true){
        if (inFlight) {
            elapsedTime = millis()-broadcastStartTime;
            broadcastStartTime = millis();
            telemetry_send();
            //set_sleep_amount();
            flash();
            threads.delay(250);
            feed_gps();
        } else {
            delegate_incoming_telemetry();
            threads.delay(100);
        }
    }
}

void telemetry_send(void){
    while (!(COMMS_SERIAL.availableForWrite()>0)){
        threads.delay(1);
    }
    transmissionSize = 0;
    printBuffer = "";

    prints(String(packetCount++));                                          // packet count
    prints(String(get_time()));                                             // current mission time

    // SYSTEM
    prints(tempmonGetTemp());                                               // internal temperature

    // GPS - WLR089u0
    if (gps.location.isValid()){
        prints(String(gps.location.age()));                                 // gps age
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
    if (bme.begin(bmeAddress)){
        prints(String(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA)));           // altitude
        prints(String(get_vertical_speed()));                               // vertical speed
        prints(String(bme.readTemperature()));                              // temperature
        prints(String(bme.readHumidity()));                                 // humidity
    }else{
        for (int i = 0; i<3; i++){
            prints("#");
        }
    }

    // MPU-6050
    if (mpu.begin(mpuAddress)){
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

    prints(String(elapsedTime));                                            // uptime
    prints(String(sleepAmount));                                            // sleep time

    // CHECKSUM
    String checksum = get_checksum(printBuffer);                            // checksum
    printBuffer += checksumIdentifier+checksum;
    transmissionSize += sizeof(printBuffer);
    transmissionSize *= 8;
    COMMS_SERIAL.println("radio tx " + string_to_hex(printBuffer) + " 1");
    Serial.println(printBuffer);
}

void delegate_incoming_telemetry(void){
    //Serial.println("checking incoming data!");
    
    if (COMMS_SERIAL.available()){
        //Serial.println("got data!!!!!!");
        flash();
        String incoming = COMMS_SERIAL.readString();
        //Serial.println("echo: "+incoming);
        
        if (incoming.startsWith(telemetryPreamble)){
            //Serial.println("checking telemetry");
            incoming = incoming.substring(telemetryPreamble.length());
            incoming = hex_to_string(incoming);
            //Serial.println("string echo: "+incoming);

            // TODO: write incoming data to sd
            if (checksum_invalid(incoming)){
                // TODO: write error code to indicate deviation from checksum
            }
            if (incoming.startsWith(commandPreamble)){
                handle_command(incoming.substring(commandPreamble.length()));
            } else{
                display_incoming_data(incoming);
            }
        }
    } else {
    }
}

/*
__syntax__: CMDxxx,123,123,... => COMMAND-CODE_ARG1_ARG2_...
1xx - RADIO

2xx - SENSORS

3xx - CONTROL
    310 - zero servo
    320 - set servo position to <position>
    321 - set servo position to <position> under <time> ms

*/

int servoPin = 4;
int servoCurrentPosition = 0;
int servoSpeed = 45;
int servoSpeedRatio = 1;
int clockwise = -1;
int servoNeutral = 93;


void servo_stop(void){
    analogWrite(servoPin,servoNeutral);
}

void servo_clockwise(int speed = 67){
    analogWrite(servoPin,speed);
}

void servo_counterclockwise(int speed = 120){
    analogWrite(servoPin,speed);
}

void set_servo_position(int position){
    //Serial.println("currentPosition: "+servoCurrentPosition);
    //Serial.println("setting position to: "+position);
    /*
    if (position>servoCurrentPosition){
        servo_clockwise();
    } else if (position<servoCurrentPosition){
        servo_counterclockwise();
    }
    */
    //threads.sleep((abs(position-servoCurrentPosition)/(servoSpeed*servoSpeedRatio)));
    servo_clockwise(position);
    threads.delay(1000);
    servo_stop();
    threads.yield();
}
void handle_command(String command){
    //Serial.println("command:"+command);
    switch (command.substring(0,3).toInt()){
        case (320):
            //Serial.println("trying to rotate");
            set_servo_position((int)command.substring(3).toInt());
            break;
    }
    
}
void servo_begin(void){
    analogWriteFrequency(servoPin,240);
    servo_stop();
}

void telemetry_begin(void){
    COMMS_SERIAL.begin(commsBaudRate);
    //Serial.begin(9600);
    sleepAmount = 1000;
    packetCount = 0;
    mission_begin();
    // TODO: write csv header to sd card
}

void display_incoming_data(String data){
}

String* string_to_array(String data){
    int dataCount = 1;
    for (unsigned int i = 0; i < data.length(); i++){
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

void set_sleep_amount(void){
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

// FLIGHT CONTROLLER
float previousAltitude=0;
uint32_t lastSampleTime=0;
float get_vertical_speed(void){
    float currentAltitude;
    if (bme.begin(bmeAddress)){
        currentAltitude = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    } else {
        currentAltitude = 0;
    }
    float verticalSpeed = (currentAltitude-previousAltitude)/(millis()-lastSampleTime);
    lastSampleTime = millis();
    return verticalSpeed;
}

void control(void){
    
}

void setup(){
    telemetry_begin();
    led_begin();
    servo_begin();
    flash();
    threads.addThread(handle_data);
    if (!inFlight){
        receive();
    }
}

void loop(){
}