#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SGP40.h>

Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
Adafruit_SGP40 sgp;

void setup(){
    Serial2.begin(115200);
    Serial.begin(9600);
    bme.begin(0x76);
    mpu.begin(0x68);
    sgp.begin(&Wire1);
    sgp.selfTest();
}

void loop(){
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure();
    float vocIndex = sgp.measureVocIndex(temperature,humidity);

    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float gx = g.gyro.x;
    float gy = g.gyro.y;
    float gz = g.gyro.z;
    float temp = t.temperature;

    
    String concatenatedData = String(temperature, HEX) + "2c" + String(humidity, HEX) + "2c" + String(pressure, HEX) + "2c" + String(ax, HEX) + "2c" + String(ay, HEX) + "2c" + String(az, HEX) + "2c" + String(gx, HEX) + "2c" + String(gy, HEX) + "2c" + String(gz, HEX) + "2c" + String(temp, HEX);
    String plainData = String(temperature) + "," + String(humidity) + "," + String(pressure) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," + String(temp) + "," + String(vocIndex);
    Serial.println("radio tx "+plainData+" 1");
    delay(250);
}
