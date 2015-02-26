#define WIRE_LIBRARY Wire     // Teensy 3.x, all Arduinos
//#define WIRE_LIBRARY TinyWireM     // Trinket

#include <math.h>
#include <SPI.h>
#include "Adafruit_BLE_UART.h"
//#include <Wire.h>
//#include <TinyWireM.h>
#include <i2c_t3.h>


//#define LED_PIN 1  // Trinket
#define LED_PIN 13
#define POWER_PIN 3

#define DEG_RAD 0.01745329251994
// bits of acode from L3GD20 library by Kevin Townshend for Adafruit

#define L3GD20_ADDRESS                (0x6B)    // 1101011
#define L3GD20_ID                     0xD4
#define L3GD20H_ID                    0xD7
#define L3GD20_SCALE                  0.00875f  // mdps/LSB   // 250 dps 
//#define L3GD20_SCALE                  0.0175f   // mdps/LSB   // 500 dps 
//#define L3GD20_SCALE                  0.07f     // mdps/LSB   // 2000 dps

#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
#define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
#define LSM303_ID                     (0b11010100)
#define LSM303_REGISTER_ACCEL_CTRL_REG1_A 0x20
#define LSM303_REGISTER_MAG_MR_REG_M   0x02

#define LSM303_REGISTER_ACCEL_OUT_X_L_A  0x28
#define LSM303_REGISTER_MAG_OUT_X_H_M  0x03
#define L3GD20_REGISTER_OUT_X_L        0x28   //            r
#define L3GD20_REGISTER_WHO_AM_I       0x0F   // 11010100   r
#define L3GD20_REGISTER_CTRL_REG1      0x20   // 00000111   rw
#define L3GD20_REGISTER_CTRL_REG2      0x21   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG3      0x22   // 00000000   rw
#define L3GD20_REGISTER_CTRL_REG4      0x23   // 00000000   rw

// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
#define twoKi  (2.0f * 0.1f) // 2 * integral gain
#define twoKp  (2.0f * 0.5f) // 2 * proportional gain
volatile float integralFBx,  integralFBy, integralFBz;
volatile float beta = 0.1f;  // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
float sampleFreq = 512.0f;  // updates dynamically every second (based on the last second) inside loop()
unsigned long lastUpdate = 0, now; // sample period expressed in milliseconds
int8_t q0sent, q1sent, q2sent, q3sent;

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
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {

	now = micros();
	sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
	lastUpdate = now;

	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	float qa, qb, qc;

	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
		float halfvx, halfvy, halfvz;
		
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		
		// Estimated direction of gravity
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (ay * halfvz - az * halfvy);
		halfey += (az * halfvx - ax * halfvz);
		halfez += (ax * halfvy - ay * halfvx);
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;  // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void quickBlink(unsigned int times){
  	delay(250);
  	for(int i = 0; i < times; i++){
		digitalWrite(LED_PIN,HIGH); delay(50);
		digitalWrite(LED_PIN,LOW); delay(50);
	}
}
void setup(){

	pinMode(LED_PIN,OUTPUT);
	pinMode(POWER_PIN,OUTPUT);
	digitalWrite(POWER_PIN,HIGH);  // power pin, for sensors

	SPI.setSCK(14);

//	Serial.begin(9600);
	WIRE_LIBRARY.begin();
	BTLEserial.begin();

	delay(1000);

	WIRE_LIBRARY.beginTransmission(L3GD20_ADDRESS);
	WIRE_LIBRARY.write((byte)L3GD20_REGISTER_WHO_AM_I);
	WIRE_LIBRARY.endTransmission();
	WIRE_LIBRARY.requestFrom((byte)L3GD20_ADDRESS, (byte)1);
	byte value = WIRE_LIBRARY.read();
	WIRE_LIBRARY.endTransmission();
	uint8_t id = value;
	if ((id != L3GD20_ID) && (id != L3GD20H_ID)) {
//		Serial.println("STOP. NOT CORRECT DEVICE");
	}
	else{
		/* Set CTRL_REG1 (0x20)
		 BIT  Symbol    Description                                   Default
		 7-6  DR1/0     Output data rate                                  00
		 5-4  BW1/0     Bandwidth selection                               00
		 3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
		 2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
		 1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
		 0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */
		WIRE_LIBRARY.beginTransmission(L3GD20_ADDRESS);
		WIRE_LIBRARY.write((byte)L3GD20_REGISTER_CTRL_REG1);
		WIRE_LIBRARY.write(0x0F);
		WIRE_LIBRARY.endTransmission();
		/* Set CTRL_REG4 (0x23)
		 BIT  Symbol    Description                                   Default
		 7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
		 6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
		 5-4  FS1/0     Full scale selection                              00
		 0  SIM       SPI Mode (0=4-Wire, 1=3-Wire)                       0 */
		WIRE_LIBRARY.beginTransmission(L3GD20_ADDRESS);
		WIRE_LIBRARY.write((byte)L3GD20_REGISTER_CTRL_REG4);
		WIRE_LIBRARY.write(0x00);
		WIRE_LIBRARY.endTransmission();
	}
	quickBlink(1);

	// ACCELEROMETER
	WIRE_LIBRARY.beginTransmission(LSM303_ADDRESS_ACCEL);
	WIRE_LIBRARY.write(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	WIRE_LIBRARY.write(0x27);
	WIRE_LIBRARY.endTransmission();
	quickBlink(2);

	// MAGNETOMETER
	WIRE_LIBRARY.beginTransmission(LSM303_ADDRESS_MAG);
	WIRE_LIBRARY.write(LSM303_REGISTER_MAG_MR_REG_M);
	WIRE_LIBRARY.write(0x00);
	WIRE_LIBRARY.endTransmission();
	quickBlink(3);
}

void loop(){
//	int a0 = analogRead(A0);
//	int a1 = analogRead(A1);
//	int a2 = analogRead(A2);
//	accelX = (a0 - 321.5) / 62.5;// / VOLTS / 255.0;  
//	accelY = (a1 - 326.5) / 64.0;// / VOLTS / 255.0;
//	accelZ = (a2 - 330.5) / 64.5;// / VOLTS / 255.0;

	uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
	// GYROSCOPE
	WIRE_LIBRARY.beginTransmission(L3GD20_ADDRESS);
	// Make sure to set address auto-increment bit
	WIRE_LIBRARY.write(L3GD20_REGISTER_OUT_X_L | 0x80);
	WIRE_LIBRARY.endTransmission();
	WIRE_LIBRARY.requestFrom((byte)L3GD20_ADDRESS, (byte)6);
	// Wait around until enough data is available
	while (WIRE_LIBRARY.available() < 6);   
	xlo = WIRE_LIBRARY.read();
	xhi = WIRE_LIBRARY.read();
	ylo = WIRE_LIBRARY.read();
	yhi = WIRE_LIBRARY.read();
	zlo = WIRE_LIBRARY.read();
	zhi = WIRE_LIBRARY.read();
	float gyroX = (int16_t)(xlo | (xhi << 8));
	float gyroY = (int16_t)(ylo | (yhi << 8));
	float gyroZ = (int16_t)(zlo | (zhi << 8));
	gyroX *= L3GD20_SCALE * DEG_RAD;  // * 0.125
	gyroY *= L3GD20_SCALE * DEG_RAD;  // * 0.125
	gyroZ *= L3GD20_SCALE * DEG_RAD;  // * 0.125

	// ACCELEROMETER
	WIRE_LIBRARY.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
	WIRE_LIBRARY.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
	WIRE_LIBRARY.endTransmission();
	WIRE_LIBRARY.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)6);
	// Wait around until enough data is available
	while (WIRE_LIBRARY.available() < 6);
	xlo = WIRE_LIBRARY.read();
	xhi = WIRE_LIBRARY.read();
	ylo = WIRE_LIBRARY.read();
	yhi = WIRE_LIBRARY.read();
	zlo = WIRE_LIBRARY.read();
	zhi = WIRE_LIBRARY.read();
	float accelX = (int16_t)(xlo | (xhi << 8)) / 16500.0;
	float accelY = (int16_t)(ylo | (yhi << 8)) / 16500.0;
	float accelZ = (int16_t)(zlo | (zhi << 8)) / 16500.0;

	// MAGNETOMETER
	WIRE_LIBRARY.beginTransmission((byte)LSM303_ADDRESS_MAG);
	WIRE_LIBRARY.write(LSM303_REGISTER_MAG_OUT_X_H_M);
	WIRE_LIBRARY.endTransmission();
	WIRE_LIBRARY.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);
	// Wait around until enough data is available
	while (WIRE_LIBRARY.available() < 6);
	// Note high before low
	xhi = WIRE_LIBRARY.read();
	xlo = WIRE_LIBRARY.read();
	zhi = WIRE_LIBRARY.read();
	zlo = WIRE_LIBRARY.read();
	yhi = WIRE_LIBRARY.read();
	ylo = WIRE_LIBRARY.read();
	// Shift values to create properly formed integer (low byte first)
	float magDataX = (int16_t)(xlo | (xhi << 8));
	float magDataY = (int16_t)(ylo | (yhi << 8));
	float magDataZ = (int16_t)(zlo | (zhi << 8)); 

	// ONBOARD LIGHT WHEN MOVING
	#define LIGHT_THRESH .2
	static unsigned char LED_STATE = 0;
	float gyroMagnitude = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
	if(!LED_STATE && gyroMagnitude > LIGHT_THRESH){
		digitalWrite(LED_PIN,HIGH);
		LED_STATE = 1;
	}
	else if (LED_STATE && gyroMagnitude <= LIGHT_THRESH) {
		digitalWrite(LED_PIN,LOW);
		LED_STATE = 0;
	}

	MadgwickAHRSupdateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

//	LOG
//	#define LOG_INTERVAL 333  // milliseconds
//	static unsigned long logLastMillis = 0;
//	if(millis() > logLastMillis + INTERVAL){
//		logLastMillis = millis();
//		Serial.print("(");	Serial.print(sampleFreq);  
//		Serial.print(")Hz\t(");	Serial.print(gyroX); 
//		Serial.print(", ");	Serial.print(gyroY);
//		Serial.print(", ");	Serial.print(gyroZ);	Serial.print(")");
//		Serial.print("\t\t(");	Serial.print(accelX); 
//		Serial.print(", ");	Serial.print(accelY);
//		Serial.print(", ");	Serial.print(accelZ);	Serial.println(")");
//		Serial.print("\t\tQ(");	Serial.print(q0);  
//		Serial.print(", ");	Serial.print(q1); 
//		Serial.print(", ");	Serial.print(q2);
//		Serial.print(", ");	Serial.print(q3);	Serial.println(")"); 
//	}


	// Tell the nRF8001 to do whatever it should be working on.
	BTLEserial.pollACI();

	// Ask what is our current status
	aci_evt_opcode_t status = BTLEserial.getState();
	// If the status changed....
	if (status != laststatus) {
		// print it out!
		if (status == ACI_EVT_DEVICE_STARTED) {
			Serial.println(F("* Advertising started"));
		}
		if (status == ACI_EVT_CONNECTED) {
			Serial.println(F("* Connected!"));
		}
		if (status == ACI_EVT_DISCONNECTED) {
			Serial.println(F("* Disconnected or advertising timed out"));
		}
		// OK set the last status change to this one
		laststatus = status;
	}

	int8_t q0min = q0 * 128;
	int8_t q1min = q1 * 128;
	int8_t q2min = q2 * 128;
	int8_t q3min = q3 * 128;

	if (status == ACI_EVT_CONNECTED) {
		if(q0min != q0sent || q1min != q1sent || q2min != q2sent || q3min != q3sent){
			q0sent = q0min;  q1sent = q1min;  q2sent = q2min;  q3sent = q3min;
			uint8_t sendbuffer[4];
			sendbuffer[0] = q1min;
			sendbuffer[1] = q2min;
			sendbuffer[2] = q3min;
			sendbuffer[3] = q0min;
			char sendbuffersize = 4;
			BTLEserial.write(sendbuffer, sendbuffersize);
//digitalWrite(LED_PIN,HIGH);
		}
//else{
//digitalWrite(LED_PIN,LOW);
//}
		// Lets see if there's any data for us!
		if (BTLEserial.available()) {
			Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
		}
		// OK while we still have something to read, get a character and print it out
		while (BTLEserial.available()) {
			char c = BTLEserial.read();
			Serial.print(c);
		}

		// Next up, see if we have any data to get from the Serial console

		if (Serial.available()) {
			// Read a line from Serial
			Serial.setTimeout(100); // 100 millisecond timeout
			String s = Serial.readString();

			// We need to convert the line to bytes, no more than 20 at this time
			uint8_t sendbuffer[20];
			s.getBytes(sendbuffer, 20);
			char sendbuffersize = min(20, s.length());

			Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

			// write the data
			BTLEserial.write(sendbuffer, sendbuffersize);
		}
	}
}
