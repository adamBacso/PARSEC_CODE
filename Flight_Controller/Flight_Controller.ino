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
// GPS
#define GPS_SERIAL      Serial1

// GUIDANCE THRESHOLDS
volatile bool gpsActive = false;
int guidanceAltitudeThreshold = 0; // m
int guidanceVSpeedThreshold = -2; // m/s
int thresholdDistanceToTarget = 10; // m
int courseDeviationThreshold = 0.5; // in degrees
float drumRadius = 1.1; // in cm
float maxPullLength = 4; // in cm
float targetLatitude;
float targetLongitude;
float zeroAltitude = 0;

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
int radioFrequency;
int syncWord;
int radioBandwidth;
int spreadingFactor;
float chirpRate;
int radioBitrate;
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

// LED
const uint8_t ledPin = 14;
const int ledDelay = 25; // in ms

///////////////////////////////////////////////////////////////////////////////////
// ~SETUP
bool go = false;

void kacat_init(void){
    led_begin();
    for (int i = 0; i<5; i++){
        delay(975);
        flash();
    }
    serial_begin();
    gpsActive = true;
    threads.addThread(feed_gps);
    Serial.println("####__KACAT_INIT__####");
    telemetry_begin();
    get_radio_info();
    sd_begin();
    servo_begin();
    gps_begin();
    guidance_begin();
    if (Serial){
        while (!go){
            capture_command();
        }
    } else {
        while (!go){
            handle_incoming_data();
        }
    }
}
    
    
    ///////////////////////////////////////////////////////////////////////////////
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

void send(String data){
    if (COMMS_SERIAL.availableForWrite()){
        COMMS_SERIAL.println("radio tx "+string_to_hex(data)+" 1");
    }
}

void receive(void){
    COMMS_SERIAL.println("radio rx 0");
}

void stop_reception(void){
    COMMS_SERIAL.println("radio rxstop");
}

void get_radio_info(){
    COMMS_SERIAL.println("radio get freq");
    while (!COMMS_SERIAL.available());
    radioFrequency = COMMS_SERIAL.readString().toInt();
    COMMS_SERIAL.println("radio get bw");
    while (!COMMS_SERIAL.available());
    radioBandwidth = COMMS_SERIAL.readString().toInt();
    COMMS_SERIAL.println("radio get sync");
    while (!COMMS_SERIAL.available());
    syncWord = COMMS_SERIAL.readString().toInt();
    COMMS_SERIAL.println("radio get sf");
    while (!COMMS_SERIAL.available());
    spreadingFactor = COMMS_SERIAL.readString().toInt();
    COMMS_SERIAL.println("radio get cr");
    while (!COMMS_SERIAL.available());
    String rawCR = COMMS_SERIAL.readString();
    chirpRate = rawCR[1] / rawCR[2];

    Serial.println("Radio info:");
    Serial.println("\tFrequency: "+radioFrequency);
    Serial.println("\tBandwidth: "+radioBandwidth);
    Serial.println("\tSync word: "+syncWord);
    Serial.println("\tSpreading Factor: "+spreadingFactor);
    Serial.println((String)"\tChirp rate: "+chirpRate);
    Serial.println("Radio: OK");
    send("\tRemote_Frequency: "+radioFrequency);
    send("\tRemote_Bandwidth: "+radioBandwidth);
    send("\tRemote_Sync word: "+syncWord);
    send("\tRemote_Spreading Factor: "+spreadingFactor);
    send((String)"\tRemote_Chirp rate: "+chirpRate);
    send("Remote_Radio: OK");
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
        Serial.print("__ERROR__: ");
        Serial.println("Card failed, or not present");
    } else {
        Serial.println("SD: OK");
    }

    int fileIndex = 1;
    while (1){
        String nameToCheck = logName+fileIndex+logType;
        if (SD.exists((nameToCheck).c_str())){
            fileIndex++;
        } else {
            logName = nameToCheck;
            Serial.println("\tFile Name: "+logName);
            break;
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
    Serial.println((String)"newTarget_OK:"+targetLatitude+","+targetLongitude);
    send((String)"Remote_newTarget_OK:"+targetLatitude+","+targetLongitude);
}

void gps_begin(void){
    Serial.print("GPS initializing");
    while (!gps.location.isUpdated()){
        Serial.print(".");
        send("waiting for gps...");
    } Serial.println();
    Serial.println("GPS info:");
    Serial.println((String)"\tCurrent latitude: "+gps.location.lat());
    Serial.println((String)"\tCurrent longitude: "+gps.location.lng());
    Serial.println((String)"\tCurrent GPS altitude: "+gps.altitude.meters());
    Serial.println("GPS: OK");
    send("Remote_GPS: OK");
}

const char *gpsStream =
    "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
    "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
    "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
    "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
    "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
    "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

void feed_gps(void){
    while (gpsActive){
        while (GPS_SERIAL.available()){
            gps.encode(GPS_SERIAL.read());
        }
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
    pinMode(ledPin,OUTPUT);
    digitalWrite(ledPin,LOW);
}

void flash(){
    digitalWrite(ledPin,HIGH);
    delay(ledDelay);
    digitalWrite(ledPin,LOW);
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
    Serial.println("Telemetry: OK");
    send("Telemetry: OK");
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
void broadcast_data(void){
    flash();
    while (1){
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
    uint16_t transmissionTime = (transmissionSize / radioBitrate) * 1000;
    sleepAmount = (transmissionTime/percentActive)*(100-percentActive);
}

int radio_bitrate(){
    radioBitrate = spreadingFactor * (radioBandwidth/pow(2,spreadingFactor)) * chirpRate;
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
        prints(String(read_altitude()));                                    // altitude
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
    transmissionSize += sizeof(string_to_hex(printBuffer));
    transmissionSize *= 8;
    send(printBuffer);
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
000 - flight ready mode

1xx - RADIO
    100 - set frequency
    101 - set sync word
    102 - set bandidth
    103 - set spreading factor
    104 - set chirp rate

    110 - set percent active

2xx - SENSORS

3xx - CONTROL
    310 - zero servo (CMD310)
    320 - set servo position to <position> (CMD320<position>)
    
    330 - set target location to current location (CMD330)
    331 - set target location (CMD331<latitude>,<longitude>)

    340 - zero altitude at current altitude

    350 - set guidance altitude threshold
    351 - set guidance vertical speed threshold
*/

void handle_command(String command){
    int commandCodeLength = 3;
    switch ((int)command.substring(0,commandCodeLength).toInt()){
        case (000):
            go = true;
            break;
        case (320):
            set_servo_position((int)command.substring(commandCodeLength).toInt());
            break;
        case (330):
            set_target(gps.location.lat(),gps.location.lng());
            break;
        case (331):
            int separatorIndex = command.substring(3).indexOf(",");
            set_target(command.substring(commandCodeLength,separatorIndex).toFloat(),command.substring(separatorIndex+1).toFloat());
            break;
        case (340):
            zero_altitude();
            break;
        case (350):
            set_guidance_altitude_threshold(command.substring(commandCodeLength).toFloat());
            break;
        case (351):
            set_guidance_vspeed_threshold(command.substring(commandCodeLength).toFloat());
            break;
        default:
            break;
    }
}

void capture_command(void){
    if (Serial.available()){
        String command = Serial.readString();
        if (command.startsWith(commandPreamble)){
            send(command);
            Serial.println(command+" sent!");
            handle_command(command.substring(3));
        }
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
    set_servo_position(lineLength/drumRadius*PI);
}


void servo_begin(void){
    analogWriteFrequency(servoPin,240);
    servo_stop();
    servo_reset();
    servo_zero();
    Serial.println("Servo:");
    Serial.println("\tClockwise speed: "+clockwise);
    Serial.println("\tCounterclockwise speed: "+counterclockwise);
    Serial.println("Servo: OK");
}

// define current rotation as ZERO
void servo_zero(void){
    servoCurrentPosition = 0;
}
// rotate servo to last zeroed location
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

void set_clockwise(int speed){
    clockwise = speed;
}

void set_counterclockwise(int speed){
    counterclockwise = speed;
}

///////////////////////////////////////////////////////////////////////////////////
// ~DESCENT CONTROL

void guidance_begin(void){
    Serial.println("Guidance info:");
    Serial.println((String)"\tTarget latitude: "+targetLatitude);
    Serial.println((String)"\tTarget longitude: "+targetLongitude);
    if (bme.begin(bmeAddress)){
        Serial.println((String)"\tCurrent altutude: "+read_altitude());
    } else {
        Serial.println("\tCurrent altitude: __bmeInitError__");
    }
    Serial.println("\tGuidance altitude threshold: "+guidanceAltitudeThreshold);
    Serial.println("\tGuidance vertical speed threshold: "+guidanceVSpeedThreshold);
    Serial.println("Guidance: OK");
}

void set_guidance_altitude_threshold(float altitude){
    guidanceAltitudeThreshold = altitude;
}

void set_guidance_vspeed_threshold(float vspeed){
    guidanceVSpeedThreshold = vspeed;
}

void confirmGuidance(void){
    while (!COMMS_SERIAL.availableForWrite()>0);
    send("__guidance active__");
}

void descent_guidance(void){
    while (read_altitude()<guidanceAltitudeThreshold); threads.delay(1000); // ensures that guidance only activates after passing threshold altitude
    while (vertical_speed()<guidanceVSpeedThreshold);
    threads.addThread(confirmGuidance);

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
        } else {
            servo_reset();
        }
        threads.yield();
    }
    threads.yield();
}


// FLIGHT CONTROLLER
double previousAltitude=0;
uint32_t lastSampleTime=0;
double vertical_speed(void){
    double currentAltitude = read_altitude();
    double verticalSpeed = (currentAltitude-previousAltitude)/(millis()-lastSampleTime);
    lastSampleTime = millis();
    return verticalSpeed;
}

float read_altitude(void){
    if (bme.begin(bmeAddress)){
        return bme.readAltitude(SEA_LEVEL_PRESSURE_HPA)-zeroAltitude;
    } else {
        return -1;
    }
}

void zero_altitude(void){
    if (bme.begin(bmeAddress)){
        zeroAltitude = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    }
}

void setup(){
    kacat_init();
    if (inFlight){
        threads.addThread(broadcast_data);
        threads.addThread(descent_guidance);
    } else {
        receive();
        handle_incoming_data();
    }
}

void loop(){
}