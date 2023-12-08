#include <TinyGPS++.h>
#include <TinyGPSPlus.h>

// *SYSTEM

// *STATES
const byte HIBERNATION = 00;
const byte PRE_FLIGHT = 01;
const byte FLIGHT_MODE = 10;

byte currentState;
bool simulationModeActivated = false;
bool simulationModeEnabled = false;

// *COMPONENTS
// **POWER MANAGEMENT
bool isGpsPowerOn = false;
bool isRadioTransceiverOn = false;
bool isServoOn = false;

// **GPS
// https://www.pjrc.com/teensy/td_libs_TinyGPS.html
// http://arduiniana.org/libraries/tinygpsplus/
// https://aprs.gids.nl/nmea/
TinyGPSPlus gps;

static const int GPS_RX_PIN = 7, GPS_TX_PIN = 8; // Serial1 pins
static const uint32_t GPS_BAUD = 9600;


// *COMMUNICATION
int baud = 9800; // bps
float frequency = 2483; // MHz
int id = 12;

void setup(){
    currentState = PRE_FLIGHT;

    // TODO: validate components
}

void loop(){
    switch (currentState){
        case HIBERNATION:
            hibernate();
            break;
        case PRE_FLIGHT:
            pre_flight_prep();
            break;
        case FLIGHT_MODE:
            flight();
            break;
    }
}

void hibernate(){
    // turn off components

    /* while (currentState == HIBERNATION){
            wait 55 seconds
            turn on radio
            send active signal

            while (elapsed time < 5sec){
                listen
                if (CMD,PWR,ACT received){
                    currentState = PRE_FLIGHT
                    break
                }
            } 
    }       
    */

}

void pre_flight_prep(){

}

void flight(){

}