#include <Wire.h> 
#include <math.h>
#include <Adafruit_L3GD20.h>

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
#define sampleFreq	512.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
  
Adafruit_L3GD20 gyro;
  
const int xInput = A2;
const int yInput = A1;
const int zInput = A0;
const int buttonPin = 2;

// Raw Ranges:
// initialize to mid-range and allow calibration to
// find the minimum and maximum for each axis
int xRawMin = 512;
int xRawMax = 512;
int yRawMin = 512;
int yRawMax = 512;
int zRawMin = 512;
int zRawMax = 512;

// Take multiple samples to reduce noise
const int sampleSize = 10;

void setup() 
{
	analogReference(EXTERNAL);
	Serial.begin(9600);
	if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
	//if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
	//if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
	{
		Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
		while (1);
	}
}

void loop() 
{
	int xRaw = ReadAxis(xInput);
	int yRaw = ReadAxis(yInput);
	int zRaw = ReadAxis(zInput);

	if (millis() < 5000){
		AutoCalibrate(xRaw, yRaw, zRaw);
	}
	else{
//		Serial.print("Raw Ranges: X: ");
//		Serial.print(xRawMin);
//		Serial.print("-");
//		Serial.print(xRawMax);
//    
//		Serial.print(", Y: ");
//		Serial.print(yRawMin);
//		Serial.print("-");
//		Serial.print(yRawMax);
//    
//		Serial.print(", Z: ");
//		Serial.print(zRawMin);
//		Serial.print("-");
//		Serial.print(zRawMax);
//		Serial.println();
//		Serial.print(xRaw);
//		Serial.print(", ");
//		Serial.print(yRaw);
//		Serial.print(", ");
//		Serial.print(zRaw);
    
		// Convert raw values to 'milli-Gs"
		long xScaled = map(xRaw, xRawMin, xRawMax, -1000, 1000);
		long yScaled = map(yRaw, yRawMin, yRawMax, -1000, 1000);
		long zScaled = map(zRaw, zRawMin, zRawMax, -1000, 1000);
  
		// re-scale to fractional Gs
		float xAccel = xScaled / 1000.0;
		float yAccel = yScaled / 1000.0;
		float zAccel = zScaled / 1000.0;
    
		gyro.read();
  
//		Serial.print("ACCEL X: ");
//		Serial.print(xAccel);
//		Serial.print("G\tY: ");
//		Serial.print(yAccel);
//		Serial.print("G\tZ: ");
//		Serial.print(zAccel);
//		Serial.print("G\t");
//		Serial.print("GYRO X: "); Serial.print((int)gyro.data.x);
//		Serial.print("\tY: "); Serial.print((int)gyro.data.y);
//		Serial.println("\tZ: "); Serial.print((int)gyro.data.z);

		MadgwickAHRSupdateIMU(xAccel, yAccel, zAccel, gyro.data.x / 300.0, gyro.data.y / 300.0, gyro.data.z / 300.0);

		Serial.print("Q0: ");
		Serial.print(q0);
		Serial.print("\tQ1: ");
		Serial.print(q1);
		Serial.print("\tQ2: ");
		Serial.print(q2);
		Serial.print("\tQ3: ");
		Serial.println(q3);
		delay(100);
	}
}

//
// Read "sampleSize" samples and report the average
//
int ReadAxis(int axisPin){
	long reading = 0;
	analogRead(axisPin);
	delay(1);
	for (int i = 0; i < sampleSize; i++){
		reading += analogRead(axisPin);
	}
	return reading/sampleSize;
}

//
// Find the extreme raw readings from each axis
//
void AutoCalibrate(int xRaw, int yRaw, int zRaw)
{
	Serial.println("Calibrate");
	if (xRaw < xRawMin) 
    	xRawMin = xRaw;
	if (xRaw > xRawMax)
		xRawMax = xRaw;  
	if (yRaw < yRawMin)
		yRawMin = yRaw;
	if (yRaw > yRawMax)
		yRawMax = yRaw;
	if (zRaw < zRawMin)
		zRawMin = zRaw;
	if (zRaw > zRawMax)
		zRawMax = zRaw;
}
