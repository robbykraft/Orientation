#include <math.h>
// i2c_t3 replacement library for Wire.h
// replace with Wire.h if not using Teensy 3.x
#include <i2c_t3.h>

#define DEG_RAD 0.01745329251994

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

#define betaDef   0.1f    // 2 * proportional gain

volatile float beta = betaDef;                // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

float sampleFreq = 512.0f;  // updates dynamically every second (based on the last second) inside loop()
unsigned long lastUpdate = 0, now; // sample period expressed in milliseconds

volatile float integralFBx,  integralFBy, integralFBz;

#define twoKi  (2.0f * 0.1f) // 2 * integral gain
#define twoKp  (2.0f * 0.5f) // 2 * proportional gain
//#define betaDef   0.1f    // 2 * proportional gain
//volatile float beta = betaDef; // 2 * proportional gain (Kp)

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

// bits of acode from L3GD20 library by Kevin Townshend for Adafruit

#define L3GD20_ADDRESS                (0x6B)        // 1101011
#define L3GD20_ID                     0xD4
#define L3GD20H_ID                    0xD7
#define L3GD20_SCALE                  0.00875f  // mdps/LSB   // 250 dps 
//#define L3GD20_SCALE                  0.0175f  // mdps/LSB   // 500 dps 
//#define L3GD20_SCALE                  0.07f  // mdps/LSB      // 2000 dps

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

void setup(){

	pinMode(13,OUTPUT);
	pinMode(2,OUTPUT);
	digitalWrite(2,HIGH);  // power pin, for sensors

//	Serial.begin(9600);
	Wire.begin();

	delay(2000);

	Wire.beginTransmission(L3GD20_ADDRESS);
	Wire.write((byte)L3GD20_REGISTER_WHO_AM_I);
	Wire.endTransmission();
	Wire.requestFrom((byte)L3GD20_ADDRESS, (byte)1);
	byte value = Wire.read();
	Wire.endTransmission();
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
		Wire.beginTransmission(L3GD20_ADDRESS);
		Wire.write((byte)L3GD20_REGISTER_CTRL_REG1);
		Wire.write(0x0F);
		Wire.endTransmission();
		/* Set CTRL_REG4 (0x23)
		 BIT  Symbol    Description                                   Default
		 7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
		 6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
		 5-4  FS1/0     Full scale selection                              00
		 0  SIM       SPI Mode (0=4-Wire, 1=3-Wire)                       0 */
		Wire.beginTransmission(L3GD20_ADDRESS);
		Wire.write((byte)L3GD20_REGISTER_CTRL_REG4);
		Wire.write(0x00);
		Wire.endTransmission();
	}

	// ACCELEROMETER
	Wire.beginTransmission(LSM303_ADDRESS_ACCEL);
	Wire.write(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	Wire.write(0x27);
	Wire.endTransmission();

	// MAGNETOMETER
	Wire.beginTransmission(LSM303_ADDRESS_MAG);
	Wire.write(LSM303_REGISTER_MAG_MR_REG_M);
	Wire.write(0x00);
	Wire.endTransmission();
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
	Wire.beginTransmission(L3GD20_ADDRESS);
	// Make sure to set address auto-increment bit
	Wire.write(L3GD20_REGISTER_OUT_X_L | 0x80);
	Wire.endTransmission();
	Wire.requestFrom((byte)L3GD20_ADDRESS, (byte)6);
	// Wait around until enough data is available
	while (Wire.available() < 6);   
	xlo = Wire.read();
	xhi = Wire.read();
	ylo = Wire.read();
	yhi = Wire.read();
	zlo = Wire.read();
	zhi = Wire.read();
	float gyroX = (int16_t)(xlo | (xhi << 8));
	float gyroY = (int16_t)(ylo | (yhi << 8));
	float gyroZ = (int16_t)(zlo | (zhi << 8));
	gyroX *= L3GD20_SCALE * DEG_RAD;  // * 0.125
	gyroY *= L3GD20_SCALE * DEG_RAD;  // * 0.125
	gyroZ *= L3GD20_SCALE * DEG_RAD;  // * 0.125

	// ACCELEROMETER
	Wire.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
	Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
	Wire.endTransmission();
	Wire.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)6);
	// Wait around until enough data is available
	while (Wire.available() < 6);
	xlo = Wire.read();
	xhi = Wire.read();
	ylo = Wire.read();
	yhi = Wire.read();
	zlo = Wire.read();
	zhi = Wire.read();
	float accelX = (int16_t)(xlo | (xhi << 8)) / 16500.0;
	float accelY = (int16_t)(ylo | (yhi << 8)) / 16500.0;
	float accelZ = (int16_t)(zlo | (zhi << 8)) / 16500.0;

	// MAGNETOMETER
	Wire.beginTransmission((byte)LSM303_ADDRESS_MAG);
	Wire.write(LSM303_REGISTER_MAG_OUT_X_H_M);
	Wire.endTransmission();
	Wire.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);
	// Wait around until enough data is available
	while (Wire.available() < 6);
	// Note high before low
	xhi = Wire.read();
	xlo = Wire.read();
	zhi = Wire.read();
	zlo = Wire.read();
	yhi = Wire.read();
	ylo = Wire.read();
	// Shift values to create properly formed integer (low byte first)
	float magDataX = (int16_t)(xlo | (xhi << 8));
	float magDataY = (int16_t)(ylo | (yhi << 8));
	float magDataZ = (int16_t)(zlo | (zhi << 8)); 

	// ONBOARD LIGHT WHEN MOVING
	#define LIGHT_THRESH .4
	float gyroMagnitude = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
	if(gyroMagnitude > LIGHT_THRESH)
		digitalWrite(13,HIGH);
	else
		digitalWrite(13,LOW);

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
}
