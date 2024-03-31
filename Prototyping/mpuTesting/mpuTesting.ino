#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
float xOffset = 0;
float yOffset = 0;
float zOffset = 0;

void calibrateRotation(int iterations = 100) {
	Serial.println("Calibrating...");
	float xMeasurementSum = 0;
	float yMeasurementSum = 0;
	float zMeasurementSum = 0;
	for (int i = 0; i < iterations; i++) {
		sensors_event_t a, g, temp;
		mpu.getEvent(&a, &g, &temp);
		xMeasurementSum += g.gyro.x;
		yMeasurementSum += g.gyro.y;
		zMeasurementSum += g.gyro.z;
		delay(10);
	}
	xOffset = xMeasurementSum / iterations;
	yOffset = yMeasurementSum / iterations;
	zOffset = zMeasurementSum / iterations;

	Serial.println(xOffset, 5);
	Serial.println(yOffset, 5);
	Serial.println(zOffset, 5);
	Serial.println("Calibration: ok");
}
void setup(void) {
	Serial.begin(9600);

	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

	

	delay(100);
	calibrateRotation(1000);
}


float truncate(float value, int digits = 1) {
	int whole = value * pow(10, digits);
	return (float)whole / pow(10, digits);
}


float absoluteRotationX = 0;
float absoluteRotationY = 0;
float absoluteRotationZ = 0;

float elapsedTime = millis()/1000;
uint32_t previousTime = 0;
void loop() {
	/* Get new sensor events with the readings */
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);

	/* Print out the values */
    /*
	Serial.print("Acceleration X: ");
	Serial.print(a.acceleration.x);
	Serial.print(", Y: ");
	Serial.print(a.acceleration.y);
	Serial.print(", Z: ");
	Serial.print(a.acceleration.z);
	Serial.println(" m/s^2");
*/	
	float x = g.gyro.x - xOffset;
	float y = g.gyro.y - yOffset;
	float z = g.gyro.z - zOffset;
/*
	x = truncate(x);
	y = truncate(y);
	z = truncate(z);
*/
	Serial.print(x);
	Serial.print(" ");
	Serial.print(y);
	Serial.print(" ");
	Serial.print(z);
    Serial.print(" ");

	elapsedTime = millis() - previousTime;
	elapsedTime /= 1000;

    absoluteRotationX += x * elapsedTime * (180/PI);
    absoluteRotationY += y * elapsedTime * (180/PI);
    absoluteRotationZ += z * elapsedTime * (180/PI);

    previousTime = millis()/1000;
    Serial.print(absoluteRotationX);
    Serial.print(" ");
    Serial.print(absoluteRotationY);
    Serial.print(" ");
    Serial.println(absoluteRotationZ);

    Serial.println("");

	previousTime = millis();

/*
	Serial.print("Temperature: ");
	Serial.print(temp.temperature);
	Serial.println(" degC");

	Serial.println("");
    */
	delay(100);
}