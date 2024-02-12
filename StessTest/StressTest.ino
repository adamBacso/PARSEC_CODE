#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SGP40.h>

Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
Adafruit_SGP40 sgp;

void setup(){
    Serial2.begin(115200);
    bme.begin(0x76);
    mpu.begin(0x68);
    sgp.begin(&Wire);
    sgp.selfTest();
}

void loop(){
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure();

    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float gx = g.gyro.x;
    float gy = g.gyro.y;
    float gz = g.gyro.z;
    float temp = t.temperature;

    
    String concatenatedData = String(temperature, HEX) + "," + String(humidity, HEX) + "," + String(pressure, HEX) + "," + String(ax, HEX) + "," + String(ay, HEX) + "," + String(az, HEX) + "," + String(gx, HEX) + "," + String(gy, HEX) + "," + String(gz, HEX) + "," + String(temp, HEX);
    Serial2.println("radio tx "+concatenatedData+" 1");
}
