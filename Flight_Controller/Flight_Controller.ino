#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <CRC32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SGP40.h>
#include <sensirion_arch_config.h>
#include <sensirion_voc_algorithm.h>
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
float courseDeviationThreshold = 0.5; // in degrees
float drumRadius = 1.1; // in cm
float maxPullLength = 4; // in cm
int maxAngle = 45;
double targetLatitude = 47.2540; // Budapest coordinates
double targetLongitude = 19.1026;
float altitudeCalibration = 0;
bool inFlight = true;

// SERVO
int servoPin = 5;
volatile int servoCurrentPosition = 0;
float servoSpeed = 0.54;
int servoSpeedRatio = 1;
int servoNeutral = 93;
float servoFrequency = 240;
int clockwise = 66;
int counterclockwise = 120;

// RADIO
int commsBaudRate = 115200;
float percentActive = 10;
int radioFrequency = 868000000; // in Hz
int syncWord = 34;
int radioBandwidth = 125;
int spreadingFactor = 7;
float chirpRate = 4.0/5.0;
int radioBitrate = 115200;
// _telemetry
const String telemetryPreamble = "radio_rx ";
const String commandPreamble = "CMD";
const char separator = ',';
const String checksumIdentifier = "fff";

// SENSORS
Adafruit_BME280 bme; int bmeAddress = 0x76;
double SEA_LEVEL_PRESSURE_HPA = (1013.25);
Adafruit_MPU6050 mpu; int mpuAddress = 0x68;
Adafruit_SGP40 sgp; int sgpAddress = 0x59;
TinyGPSPlus gps; int gpsBaud = 9600;

int batteryVoltagePin = 1;

// SD CARD
uint8_t chipSelect = 10;
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
    serial_begin();
    gpsActive = true;
    threads.addThread(feed_gps);
    delay(10000); // wait for connection with the computer
    Serial.println(F("KACAT Mission Control"));
    Serial.print(F("Waiting for RADIO handshake"));
    sd_begin();
    config();
    auto_radio_setup();
    telemetry_begin();
    get_radio_info();
    servo_begin();
    gps_begin();
    callibrate_altitude();
    guidance_begin();
    //if (!Serial){
        inFlight = true;
    Serial.println("Starting flight");
    //} else {
    //    receive();
    //    inFlight = false;
    //}
}

void config(void) {
    File config = SD.open("config.txt", FILE_READ);
    if (config){
        config.read(); // 1
        config.read(); // 2
        radioFrequency = (int)config.readString().toInt(); // 3
        config.read(); // 4
        syncWord = (int)config.readString().toInt(); // 5
        config.read(); // 6
        radioBandwidth = (int)config.readString().toInt(); // 7
        config.read(); // 8
        spreadingFactor = (int)config.readString().toInt(); // 9
        config.read(); // 10
        courseDeviationThreshold = config.readString().toFloat(); // 11
        config.read(); // 12
        guidanceAltitudeThreshold = (int)config.readString().toInt(); // 13
        config.read(); // 14
        targetLatitude = (double)config.readString().toFloat(); // 15
        config.read(); // 16
        targetLongitude = (double)config.readString().toFloat(); // 17
        config.close();
    }
    else{
        config.close();
        fatal_error("Config file not found.");
    }
}

void fatal_error(const char* message){
    Serial.println("ERROR: " + *message);
    while (1);
}
    
///////////////////////////////////////////////////////////////////////////////
// ~SERIAL

void serial_begin(void){
    COMMS_SERIAL.begin(commsBaudRate); // LORA RADIO
    Serial.begin(9600); // USB SERIAL
    GPS_SERIAL.begin(gpsBaud); // GPS
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
    COMMS_SERIAL.println("radio tx "+data+" 1");
    sd_write(data);
    COMMS_SERIAL.clear();
}

void receive(void){
    COMMS_SERIAL.clear();
    COMMS_SERIAL.println("radio rx 0");
    while (!COMMS_SERIAL.available());
    Serial.println(String("reception status")+COMMS_SERIAL.readString());
    delay(50);
    COMMS_SERIAL.clear();
}

void stop_reception(void){
    COMMS_SERIAL.println("radio rxstop");
}

void get_radio_info(){
    delay(500);
    COMMS_SERIAL.clear();
    Serial.print(F("Frequency: "));
    send_command("radio get freq");
    delay(500);
    Serial.print(F("Bandwidth: "));
    send_command("radio get bw");
    delay(500);
    Serial.print(F("Sync word: "));
    send_command("radio get sync");
    delay(500);
    Serial.print(F("Spreading factor: "));
    send_command("radio get sf");
    delay(500);
    Serial.print(F("Chirp rate: "));
    send_command("radio get cr");

    Serial.println("Radio info:");
}

void radio_begin(void){
    stop_reception();
    while (COMMS_SERIAL.available()<=0);
    COMMS_SERIAL.clear();
}

void manual_radio_setup(void){
    Serial.println(COMMS_SERIAL.readString());
    Serial.println(F("MANUAL_RADIO_SETUP"));
    String serialCommand = "";
    while (serialCommand != "q"){
        if (Serial.available()>0){
            serialCommand = Serial.readString();
            if (serialCommand == "q"){
                break;
            }
            COMMS_SERIAL.println(serialCommand);
        }
        else if (COMMS_SERIAL.available()>0){
            Serial.println(COMMS_SERIAL.readString());
        }
    }
}
void send_command(String command){
    Serial.println(command);
    COMMS_SERIAL.println(command);
    while (COMMS_SERIAL.available()<=0);
    Serial.println(COMMS_SERIAL.readString());
}

void auto_radio_setup(void){
    led_begin();
    flash();
    Serial.println(COMMS_SERIAL.readString());
    Serial.println(F("AUTOMATIC_RADIO_SETUP"));
    send_command(String("radio rxstop"));
    send_command(String("radio set freq "+String(radioFrequency)));
    send_command(String("radio set bw "+String(radioBandwidth)));
    send_command(String("radio set sf "+String(spreadingFactor)));
    send_command(String("radio set sync "+String(syncWord)));
    delay(1000);
    COMMS_SERIAL.clear();
}

///////////////////////////////////////////////////////////////////////////////////
// ~SD CARD

int get_chipSelect(void){
    return chipSelect;
}

void sd_begin(void){
    pinMode(chipSelect, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, INPUT);
    pinMode(13, OUTPUT);
    digitalWrite(chipSelect,HIGH);
    Serial.print("Initializing SD card...");
    //pinMode(chipSelect,OUTPUT);

    // see if the card is present and can be initialized:
    if (SD.begin(chipSelect)) {
        Serial.println("SD: OK");
        int fileIndex = 1;
        while (SD.exists((logName+fileIndex+logType).c_str())){
            fileIndex++;
        }
        logName = logName+fileIndex+logType;
        Serial.println((String)"\tFile Name: "+logName);
        sd_write("");
    } else {
        Serial.print("__ERROR__: ");
        Serial.println("Card failed, or not present");
        fatal_error("SD: FAIL");
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
    send((String)"$Remote_newTarget_OK:"+targetLatitude+","+targetLongitude);
}

volatile bool gpsAcquisitionTerminate = false;
void gps_begin(void){
    Serial.print("GPS initializing");
    while ((!gps.location.isUpdated())||(Serial.available()>0)){
        Serial.print(".");
        flash();
        delay(500);
    } Serial.println();
    Serial.println("GPS info:");
    Serial.println((String)"\tCurrent latitude: "+gps.location.lat());
    Serial.println((String)"\tCurrent longitude: "+gps.location.lng());
    Serial.println((String)"\tCurrent GPS altitude: "+gps.altitude.meters());
    Serial.println("GPS: OK");
}

const char *gpsStream =
    "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
    "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
    "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
    "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
    "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
    "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

// runs continously so that the gps is continously fed
void feed_gps(void){
    while (gpsActive){
        while (GPS_SERIAL.available()){
            if (gps.encode(GPS_SERIAL.read())){
            }
        }
        /*
        if (*gpsStream){
            gps.encode(*gpsStream++);
        }
        */
        threads.delay(250);
    }
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

int packetCount = 0;

float sleepAmount;

String printBuffer = "";
String radioTelemetry = "";
int transmissionSize;

void telemetry_begin(void){
    sleepAmount = 1000;
    packetCount = 0;
    mission_begin();
    stop_reception();
    Serial.println("Telemetry: OK");
    send("$Telemetry: OK");
}

void prints(String data){
    String dataBlock = data + separator;
    printBuffer += dataBlock;
}

void prints(int data){
    printBuffer += String(data) + separator;
    radioTelemetry += get_pentadecimal(data) + "f";
}

void prints(float data, int precision = 0){
    printBuffer += String(data) + separator;
    radioTelemetry += get_pentadecimal(data, precision) + "f";
}

void prints(double data){
    printBuffer += String(data) + separator;
    radioTelemetry += get_pentadecimal((float)data) + "f";
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
    //flash();
    while (1){
        //String gpsData = GPS_SERIAL.readString();
        //if (gpsData.startsWith("$GPGGA")){
            //Serial.println(gpsData);
        //}
        telemetry_send();
        set_sleep_amount();
        threads.delay(sleepAmount);
    }
}

void set_sleep_amount(void){
    uint16_t transmissionTime = (transmissionSize / radioBitrate) * 1000;
    sleepAmount = (transmissionTime/percentActive)*(100-percentActive);
}

void radio_bitrate(void){
    radioBitrate = spreadingFactor * (radioBandwidth*1000/pow(2,spreadingFactor)) * chirpRate;
}

const String telemetryDataNames = "packetCount,missionTime,internalTemperature,gpsAge,latitude,longitude,courseToTarget,distanceToTarget,currentCourse,gpsAltitude,barometricAltitude,bmeTemperature,humidity,accelerationX,accelerationY,accelerationZ,gyroscopeX,gyroscopeY,gyroscopeZ,mpuTemperature,sgpRawVoc,sgpVocIndex,uptime,sleepTime,checksum,transmissionSize";
void telemetry_send(void){
    //while (!(COMMS_SERIAL.availableForWrite()>0)){
    //    threads.delay(2);
    //}
    transmissionSize = 0;
    printBuffer = "";
    radioTelemetry = "";

    // HEADER

    prints(packetCount++);                                          // packet count
    prints(get_time(),2);                                             // current mission time

    // SYSTEM
    prints(tempmonGetTemp(),1);                                               // internal temperature

    // GPS
    if (gps.location.isValid()){
        prints(gps.location.lat(),6);                                 // latitude
        prints(gps.location.lng(),6);                                 // longitude
        prints(course_to_target(),6);                                 // course to target
    }else{
        for (int i = 0; i<5; i++){
            prints("a");
        }
    }

    if (gps.course.isValid()){
        prints(gps.course.deg(),6U);                                   // current course
    }else{
        prints("a");
    }

    if (gps.altitude.isValid()){
        prints(gps.altitude.meters(),3);                              // gps altitude
    }else{
        prints("a");
    }
    // BME280
    float temperature = 0;
    float humidity = 0;
    float pressure = 0;
    if (bme.begin(bmeAddress)){
        pressure = bme.readPressure()/100;
        temperature = bme.readTemperature();
        humidity = bme.readHumidity();
        prints(pressure,4);                                    //     pressure in hPa
        prints(temperature,2);       
        prints(humidity,2);                                           // humidity
    }
    else{
        for (int i = 0; i<3; i++){
            prints("a");
        }
    }

    // MPU-6050
    if (mpu.begin(mpuAddress)){
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        // acceleration
        prints(a.acceleration.x,3);                                   // acceleration x
        prints(a.acceleration.y,3);                                   // acceleration y
        prints(a.acceleration.z,3);                                   // acceleration z
        // gyroscope
        prints(g.gyro.x,3);                                           // gyro x
        prints(g.gyro.y,3);                                           // gyro y
        prints(g.gyro.z,3);                                           // gyro z
    }// temperature
    else{
        for (int i = 0; i<6; i++){
            prints("a");
        }
    }

    // SGP40
    bool sgpAvailable = sgp.begin(&Wire1);
    //Serial.println((String)"SGP state: "+sgpAvailable);
    if (sgpAvailable){
    prints((int)sgp.measureVocIndex(temperature,humidity));          // voc index based on temperature and humidity
    } else {
        for (int i = 0; i<2; i++){
            prints("a");
        }
    }
    prints(sleepAmount,3);                                            // sleep time

    // CHECKSUM
    String checksum = get_checksum(radioTelemetry);                            // checksum
    radioTelemetry += checksumIdentifier+checksum;
    transmissionSize += printBuffer.length();
    transmissionSize *= 8;
    prints(transmissionSize);                                      // transmission size
    send(radioTelemetry);
    Serial.println(printBuffer);
}

String get_checksum(String data){
    int checksum = 0;
    for (size_t i = 0; i < data.length(); ++i) {
        checksum ^= static_cast<int>(data.charAt(i));
    }
    return String(checksum);
}

String get_pentadecimal(int number){
    return String(number, 15);
}
String get_pentadecimal(float number, int precision = 0){
    int wholePart = (int)number;
    float decimalNumber = (number - wholePart);
    int decimalPlaces = get_decimal_places(number);
    if (decimalPlaces > precision){
        decimalPlaces = precision;
    }
    Serial.println((String)"Decimal places: "+String(decimalPlaces));
    int decimalPart = decimalNumber * pow(10, decimalPlaces);

    Serial.println((String)"Decimal part: "+String(decimalPart));
    String output = "";
    output += get_pentadecimal(wholePart);
    if (precision > 0){
        output += "ff";
        for (int i = 0; i < get_leading_zeros(decimalPart, decimalPlaces); i++){
            output += "0";
        }
        output += get_pentadecimal(decimalPart);
    }
    return output;

}

int get_decimal_places(float number){
    int counter = 0;
    while (number != (int)number){
        number *= 10;
        counter++;
    }
    return counter;
}

int get_leading_zeros(int number, int decimalPlaces){
    int target = pow(10, decimalPlaces-1);
    int counter = 0;
    while (target-number>0){
        number *= 10;
        counter++;
    }
    return counter;
}

///////////////////////////////////////////////////////////////////////////////////
// ~INCOMING DATA HANDLING

void handle_incoming_data(void){
    flash();
    receive();
    while (true){
        delegate_incoming_telemetry();
        threads.delay(5);
    }
}
void delegate_incoming_telemetry(void){
    String incoming;
    Serial.println(COMMS_SERIAL.readString());
    
    if (COMMS_SERIAL.available()>0){
        incoming = COMMS_SERIAL.readString();
        
        if (incoming.startsWith(telemetryPreamble)){
            incoming = incoming.substring(telemetryPreamble.length());
            incoming = hex_to_string(incoming);

            display_incoming_data(incoming);
        }
    } else if (Serial.available()) {
        
    }
}

void display_incoming_data(String data){ // TODO: do something already about this!!!!
    Serial.println(data);
}

///////////////////////////////////////////////////////////////////////////////////
// ~COMMAND

/*
__syntax__: CMDxxx123,123,... => COMMAND-CODEARG1,ARG2,...
000 - flight ready mode (CMD000)
900 - terminate gps acquisition (CMD900)

1xx - RADIO
    100 - set frequency (CMD100<feq>)
    101 - set sync word (CMD101<syncWord>)
    102 - set bandidth (CMD102<bandwidth>)
    103 - set spreading factor (CMD103<spreadingFactor>)
    104 - set chirp rate (CMD104<chirpRate>)

    110 - set percent active (CMD110<percentActive>)

2xx - SENSORS
    201 - internal temperature
    210 - bme available
    211 - bme temperature
    212 - bme humidity
    213 - bme pressure
    214 - bme altitude

    220 - mpu available
    221 - mpu temperature

    230 - sgp available
    231 - sgp raw measurement
    232 - sgp VOC index

    291 - set pressure at sea level

3xx - CONTROL
    310 - zero servo (CMD310)
    311 - set servo clockwise speed
    312 - set servo counterclockwise speed
    320 - set servo position to <position> (CMD320<position>)
    
    330 - set target location to current location (CMD330)
    331 - set target location (CMD331<latitude>,<longitude>)

    340 - zero altitude at current altitude (CMD340<zeroAltitude>)

    350 - set guidance altitude threshold (CMD350<altitude>)
    351 - set guidance vertical speed threshold (CMD351<vspeed>)
*/

void handle_command(String command){
    int commandCodeLength = 3;
    int identifier = command.substring(0, commandCodeLength).toInt();
    String commandArgs = command.substring(commandCodeLength);
    int separatorIndex;
    Serial.println(F("Handling data"));
    
    switch (identifier) {
        case 0: { // flight ready mode
            go = true;
            break;
        }
        case 100: { // set frequency
            radioFrequency = commandArgs.toInt();
            send_request("radio set freq " + String(radioFrequency));
            break;
        }
        case 101: { // set sync word
            syncWord = commandArgs.toInt();
            send_request("radio set sync " + String(syncWord));
            break;
        }
        case 102: { // set bandwidth
            radioBandwidth = commandArgs.toInt();
            send_request("radio set bw " + String(radioBandwidth));
            break;
        }
        case 103: { // set spreading factor
            spreadingFactor = commandArgs.toInt();
            send_request("radio set sf " + String(spreadingFactor));
            break;
        }
        case 104: { // set chirp rate
            chirpRate = commandArgs.toFloat();
            // Assuming chirp rate is set via a custom command or calculation as it's not directly supported by WLR089U0 commands
            break;
        }
        case 110: { // set percent active
            percentActive = commandArgs.toFloat();
            break;
        }
        case 201: {
            String message = (String)"Internal temperature: "+tempmonGetTemp();
            Serial.println(message);
            send("$Remote: "+message);
            break;
        }
        case 210: {
            String message = (String)"BME280 available: "+bme.begin(bmeAddress);
            Serial.println(message);
            send("$Remote: "+message);
            break;
        }
        case 211: {
            String message =(String)"BME temperature: "+bme.readTemperature();
            Serial.println(message);
            send("$Remote: "+message);
            break;
        }
        case 212: {
            String message =(String)"Humidity: "+bme.readHumidity();
            Serial.println(message);
            send("$Remote: "+message);
            break;
        }
        case 213: {
            String message =(String)"Pressure (Pa): "+bme.readPressure();
            Serial.println(message);
            send("$Remote: "+message);
            break;
        }
        case 214: {
            String message =(String)"BME altitude: "+bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
            Serial.println(message);
            send("$Remote: "+message);
            break;
        }
        case 230: {
            String message = (String)"SGP40 available: "+sgp.begin(&Wire1);
            Serial.println(message);
            message = (String)"SGP40 test success: "+sgp.selfTest();
            Serial.println(message);
            break;
        }
        case 231: {
            String message = (String)"Raw VOC measurement: "+sgp.measureRaw(bme.readTemperature(),bme.readHumidity());
            Serial.println(message);
            send("$Remote: "+message);
            break;
        }
        case 232: {
            break;
        }
        case 311: {
            set_clockwise((int)commandArgs.toInt());
            break;
        }
        case 312: {
            set_counterclockwise((int)commandArgs.toInt());
            break;
        }
        case 320: { // set servo position
            set_servo_position((int)commandArgs.toInt());
            break;
        }
        case 330: { // set target location to current location
            set_target(gps.location.lat(), gps.location.lng());
            break;
        }
        case 331: { // set target location
            separatorIndex = commandArgs.indexOf(',');
            double arg1 = commandArgs.substring(0, separatorIndex).toFloat();
            double arg2 = commandArgs.substring(separatorIndex + 1).toFloat();
            set_target(arg1, arg2);
            break;
        }
        case 350: { // set guidance altitude threshold
            guidanceAltitudeThreshold = (int)commandArgs.toInt();
            Serial.println(guidanceAltitudeThreshold);
            Serial.println((String)"Guidance altitude threshold: "+guidanceAltitudeThreshold);
            send((String)"$Remote_Guidance altitude threshold: "+guidanceAltitudeThreshold);
            break;
        }
        case 351: { // set guidance vertical speed threshold
            guidanceVSpeedThreshold = (int)commandArgs.toInt();
            Serial.println((String)"Guidance vertical speed thresholt: "+guidanceVSpeedThreshold);
            send((String)"$Remote_Guidance vertical speed threshold: "+guidanceVSpeedThreshold);
            break;
        }
        case 900: { // terminate gps acquisition
            gpsAcquisitionTerminate = true;
            Serial.println(F("GPS terminated"));
            break;
        }
        default: {
            // Handle unknown command
            Serial.println("Unknown command: " + command);
            send("$Remote_Unknown command!");
            break;
        }
    }
}

void capture_command(){
    if (Serial.available()){
        String command = Serial.readString();
        if (command.startsWith(commandPreamble)){
            send(command);
            handle_command(command.substring(3));
        }
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

void set_servo_position(int targetPosition){
    if (abs(servoCurrentPosition+targetPosition) < 50){
        int amountToRotate = targetPosition - servoCurrentPosition;
        int servoSleep = abs(amountToRotate / servoSpeed);
        rotate_servo(amountToRotate);
        threads.delay(servoSleep);
        servo_stop();
        servoCurrentPosition += amountToRotate;
    }
    threads.yield();
}

void pull_line_amount(float lineLength){
    set_servo_position(lineLength/drumRadius*PI);
}


void servo_begin(void){
    analogWriteFrequency(servoPin,servoFrequency);
    servo_zero();
    Serial.println("Servo:");
    Serial.println((String)"\tClockwise speed: "+clockwise);
    Serial.println((String)"\tCounterclockwise speed: "+counterclockwise);
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
    while (true){
        set_servo_position(60);
        delay(1000);
        set_servo_position(-60);
        delay(1000);
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
    Serial.println(bme.begin(bmeAddress));
    if (bme.begin(bmeAddress)){
        Serial.println((String)"\tCurrent altutude: "+read_altitude());
    } else {
        Serial.println("\tCurrent altitude: __bmeInitError__");
    }
    Serial.println("\tGuidance altitude threshold: "+guidanceAltitudeThreshold);
    Serial.println("\tGuidance vertical speed threshold: "+guidanceVSpeedThreshold);
    Serial.println("Guidance: OK");
}

void tilt_controlled_servo(void){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (g.gyro.x > 0){
        set_servo_position(maxAngle);
    }else{
        set_servo_position(maxAngle*-1);
    }
}

void dummy_test_flight(){
    while (1){
        set_servo_position(maxAngle);
        threads.delay(5000);
        set_servo_position(-maxAngle);
        threads.delay(5000);
    }
}

void set_guidance_altitude_threshold(float altitude){
    guidanceAltitudeThreshold = altitude;
}

void set_guidance_vspeed_threshold(float vspeed){
    guidanceVSpeedThreshold = vspeed;
}

void confirmGuidance(void){
    while (!(COMMS_SERIAL.availableForWrite()>0));
    send("__guidance active__");
    threads.yield();
}

void descent_guidance(void){
    float baseAltitude = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    while (bme.readAltitude(SEA_LEVEL_PRESSURE_HPA)-baseAltitude<guidanceAltitudeThreshold){    // are we flying? FIXME: chamber may be airtight -> constant pressure
                                                                                                // TODO: look into using accelerometer
        threads.delay(250);
    }

    bool guidanceNeeded = true;
    while (guidanceNeeded){
        if (gps.location.isValid()){ // are we getting good data?
            if (gps.distanceBetween(gps.location.lat(),gps.location.lng(),targetLatitude,targetLongitude)>thresholdDistanceToTarget){ // are we there yet?
                int32_t courseDeviation = course_to_target()-gps.course.value();
                Serial.println(courseDeviation);
                if (abs(courseDeviation) > courseDeviationThreshold){ // are we headed in the right direction?
                    if (courseDeviation > 0){ // FIXME: decide which way is positive and negative
                        set_servo_position(maxAngle);
                    } else {
                        set_servo_position(maxAngle*-1);
                    }
                } else {
                    servo_reset();
                }
            } else {
                set_servo_position(maxAngle);
                guidanceNeeded = false;
            }
        } else {
            servo_reset();
        }
        threads.delay(50);
    }
}

void callibrate_altitude(void){
    altitudeCalibration = gps.altitude.meters() - bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
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
        return bme.readAltitude(SEA_LEVEL_PRESSURE_HPA)+altitudeCalibration;
    } else {
        return -1;
    }
}

///////////////////////////////////////////////////////////////////////////////////
// SYSTEM
float read_battery_voltage(void){
    int sensorValue = analogRead(batteryVoltagePin);
    return sensorValue * (5.0 / 1023.0);
}

///////////////////////////////////////////////////////////////////////////////////
// BODY
void setup(){
    sgp.begin(&Wire1);
    delay(100);
    sgp.selfTest();
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