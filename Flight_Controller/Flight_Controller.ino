#include <PWMServo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
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
// GPS
#define GPS_SERIAL      Serial1

// GUIDANCE THRESHOLDS
int guidanceAltitudeThreshold = 0; // m
int guidanceVSpeedThreshold = -2; // m/s
int thresholdDistanceToTarget = 10; // m
int courseDeviationThreshold = 0.5; // in degrees
float drumRadius = 1.1; // in cm
float maxPullLength = 4; // in cm
float targetLatitude;
float targetLongitude;

// SERVO
int servoPin = 4;
int servoCurrentPosition = 0;
float servoSpeed = 0.54;
int servoSpeedRatio = 1;
int servoNeutral = 93;
int clockwise = 67;
int counterclockwise = 120;

// RADIO
int commsBaudRate = 115200;
float percentActive = 10;
int radioFrequency = 868000000;
int syncWord;
int bandwidth;
// _telemetry
const String telemetryPreamble = "radio_rx ";
const String commandPreamble = "CMD";
const char separator = ',';
const char checksumIdentifier = '*';

// SENSORS
Adafruit_BME280 bme; int bmeAddress = 0x76;
const double SEA_LEVEL_PRESSURE_HPA = (1013.25);
Adafruit_MPU6050 mpu; int mpuAddress = 0x68;
TinyGPSPlus gps; int gpsBaud = 9600;

// SD CARD
uint8_t chipSelect = 10U;
File flightLog;
String logName = "flightLog";
const String logType = ".txt";

///////////////////////////////////////////////////////////////////////////////////
// ~SERIAL

void serial_begin(void){
    COMMS_SERIAL.begin(commsBaudRate);
    Serial.begin(9600);
    GPS_SERIAL.begin(gpsBaud);
}

///////////////////////////////////////////////////////////////////////////////////
// ~RADIO

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

///////////////////////////////////////////////////////////////////////////////////
// ~SD CARD

int get_chipSelect(void){
    return chipSelect;
}

void sd_begin(void){
    Serial.print("Initializing SD card...");
    pinMode(chipSelect,OUTPUT);

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1) {
            // No SD card, so don't do anything more - stay stuck here
        }
    }
    Serial.println("card initialized.");

    bool fileUnique = false;
    int fileIndex = 1;
    while (!fileUnique){
        
        String nameToCheck = logName+fileIndex+logType;
        if (SD.exists((nameToCheck).c_str())){
            Serial.println(nameToCheck + " already exists!");
            fileIndex++;
        } else {
            logName = logName+fileIndex+logType;
            fileUnique = true;
        }

    }
}

void sd_write(String data){
    flightLog = SD.open(logName.c_str(), FILE_WRITE);
    if (flightLog){
        flightLog.println(data.c_str());
    }
    flightLog.close();
}

///////////////////////////////////////////////////////////////////////////////////
// ~GPS

void set_target(double latitude, double longitude){
    targetLatitude = latitude;
    targetLongitude = longitude;
}

const char *gpsStream =
    "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
    "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
    "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
    "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
    "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
    "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

void feed_gps(void){
    if (GPS_SERIAL.available()){
        gps.encode(GPS_SERIAL.read());
    }
    /*
    if (*gpsStream){
        encode(*gpsStream++);
    }
    */
}

double distance_to_target(void){
    return gps.distanceBetween(gps.location.lat(),gps.location.lng(),targetLatitude,targetLongitude);
}


double course_to_target(void){
    return gps.courseTo(gps.location.lat(),gps.location.lng(),targetLatitude,targetLongitude);
}

///////////////////////////////////////////////////////////////////////////////////
// ~LED

void led_begin(void){
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
}

void flash(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(25);
    digitalWrite(LED_BUILTIN,LOW);
}

///////////////////////////////////////////////////////////////////////////////////
// ~TIMER

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
    return float(millis()-missionStartTime)/1000;
}

void true_sleep(int milli){
    delay(milli - (millis()-processStart));
    processStart = millis();
}

///////////////////////////////////////////////////////////////////////////////////
// ~TELEMETRY

uint32_t broadcastStartTime = 0;
bool inFlight = true;

int packetCount = 0;

float sleepAmount;

String printBuffer = "";
float transmissionSize;

void telemetry_begin(void){
    sleepAmount = 1000;
    packetCount = 0;
    mission_begin();
}

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
        elapsedTime = millis()-broadcastStartTime;
        if (elapsedTime >= broadcastStartTime + sleepAmount){
            broadcastStartTime = millis();
            telemetry_send();
            set_sleep_amount();
            flash();
            //delay(100);
            feed_gps();
        }
        threads.yield();
    }
}

void set_sleep_amount(void){
    uint16_t transmissionTime = (transmissionSize / commsBaudRate) * 1000;
    sleepAmount = (transmissionTime/percentActive)*(100-percentActive);
}

void telemetry_send(void){
    while (!(COMMS_SERIAL.availableForWrite()>0)){
        //threads.delay(1);
        delay(1);
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
        prints(String(course_to_target()));                                 // course to target
        prints(String(distance_to_target()));                               // distance to target
    }else{
        for (int i = 0; i<5; i++){
            prints("#");
        }
    }

    if (gps.course.isValid()){
        prints(String(gps.course.deg()));                                   // current course
    }else{
        prints("#");
    }

    if (gps.altitude.isValid()){
        prints(String(gps.altitude.meters()));                              // gps altitude
    }else{
        prints("#");
    }
    
    // BME280
    if (bme.begin(bmeAddress)){
        prints(String(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA)));           // altitude
        prints(String(vertical_speed()));                               // vertical speed
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
    sd_write(printBuffer);
}

String get_checksum(String data){
    unsigned int checksum = CRC32::calculate(data.c_str(), data.length());
    char checksumStr[3];
    snprintf(checksumStr, sizeof(checksumStr), "%02X", checksum);
    return String(checksumStr);
}

///////////////////////////////////////////////////////////////////////////////////
// ~INCOMING DATA HANDLING

void handle_incoming_data(void){
    flash();
    while (true){
        delegate_incoming_telemetry();
        //threads.yield();
    }
}
void delegate_incoming_telemetry(void){
    //Serial.println("checking incoming data!");
    String incoming;
    
    if (COMMS_SERIAL.available()){
        //Serial.println("got data!!!!!!");
        flash();
        delay(1000);
        flash();
        incoming = COMMS_SERIAL.readString();
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
    } else if (Serial.available()) {
        
    }
}

void display_incoming_data(String data){
}

///////////////////////////////////////////////////////////////////////////////////
// ~COMMAND

/*
__syntax__: CMDxxx123,123,... => COMMAND-CODEARG1,ARG2,...
1xx - RADIO

2xx - SENSORS

3xx - CONTROL
    310 - zero servo
    320 - set servo position to <position>
    321 - set servo position to <position> under <time> ms
*/

void handle_command(String command){
    //Serial.println("command:"+command);
    switch (command.substring(0,3).toInt()){
        case (320):
            //Serial.println("trying to rotate");
            set_servo_position((int)command.substring(3).toInt());
            break;
    }
    
}

bool checksum_invalid(String data){
    String checksum = data.substring(data.indexOf(checksumIdentifier));
    if (get_checksum(data)==checksum){
        return false;
    } else {
        return true;
    }
}

///////////////////////////////////////////////////////////////////////////////////
// ~SERVO

void servo_stop(void){
    analogWrite(servoPin,servoNeutral);
}

void rotate_servo(int angle){
    if (angle > 0){
        analogWrite(servoPin,counterclockwise);
    }else {
        analogWrite(servoPin,clockwise);
    }
}

void set_servo_position(int position){
    if (abs(servoCurrentPosition) < 360){
        int amountToRotate = position - servoCurrentPosition;
        Serial.println(amountToRotate);
        rotate_servo(amountToRotate);
        float servoSleep = amountToRotate / servoSpeed;
        Serial.println(servoSleep);
        threads.delay(servoSleep);
        servo_stop();
    }
    threads.yield();
}

void pull_line_amount(float lineLength){
    set_servo_position(lineLength/drumRadius);
}


void servo_begin(void){
    Serial.println("servo_begin");
    analogWriteFrequency(servoPin,240);
    servo_stop();
    servo_reset();
}

void servo_zero(void){
    servoCurrentPosition = 0;
}
void servo_reset(){
    set_servo_position(0);
}

void servo_test(int iterations = 1){
    for (int i = 0; i<iterations; i++){
        set_servo_position(20);
        delay(1000);
        set_servo_position(-40);
        delay(1000);
        pull_line_amount(2);
        delay(1000);
        servo_reset();
        delay(1000);
        pull_line_amount(-3);
        delay(1000);
        servo_reset();
        delay(2000);
    }
}

///////////////////////////////////////////////////////////////////////////////////
// ~DESCENT CONTROL

void descent_guidance(void){
    while (vertical_speed()<guidanceVSpeedThreshold && barometric_altitude()>guidanceAltitudeThreshold);

    bool guidanceNeeded = true;
    while (guidanceNeeded){
        if (gps.location.isValid()){
            if (gps.distanceBetween(gps.location.lat(),gps.location.lng(),targetLatitude,targetLongitude)>thresholdDistanceToTarget){
                int32_t courseDeviation = course_to_target()-gps.course.value();
                if (abs(courseDeviation) > courseDeviationThreshold){
                    if (courseDeviation > 0){
                        pull_line_amount(maxPullLength);
                    } else {
                        pull_line_amount(-maxPullLength);
                    }
                } else {
                    servo_reset();
                }
            } else {
                pull_line_amount(maxPullLength);
                guidanceNeeded = false;
            }
        }
        threads.yield();
    }
    threads.yield();
}


// FLIGHT CONTROLLER
float previousAltitude=0;
uint32_t lastSampleTime=0;
float vertical_speed(void){
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

float barometric_altitude(void){
    if (bme.begin(bmeAddress)){
        return bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    } else {
        return -1;
    }
}



void setup(){
    serial_begin();
    telemetry_begin();
    led_begin();
    sd_begin();
    servo_begin();
    Serial.println("servo_init_done");
    flash();
    if (inFlight){
        //threads.addThread(handle_data);
        //handle_data();
        threads.addThread(descent_guidance);
    } else {
        receive();
        handle_incoming_data();
    }
}

void loop(){
    telemetry_send();
}
