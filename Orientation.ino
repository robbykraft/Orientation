#include <Wire.h>
#include <math.h>

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

typedef enum
{                                               // DEFAULT    TYPE
	L3GD20_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
	L3GD20_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
	L3GD20_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
	L3GD20_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
	L3GD20_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
	L3GD20_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
	L3GD20_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
	L3GD20_REGISTER_OUT_TEMP            = 0x26,   //            r
	L3GD20_REGISTER_STATUS_REG          = 0x27,   //            r
	L3GD20_REGISTER_OUT_X_L             = 0x28,   //            r
	L3GD20_REGISTER_OUT_X_H             = 0x29,   //            r
	L3GD20_REGISTER_OUT_Y_L             = 0x2A,   //            r
	L3GD20_REGISTER_OUT_Y_H             = 0x2B,   //            r
	L3GD20_REGISTER_OUT_Z_L             = 0x2C,   //            r
	L3GD20_REGISTER_OUT_Z_H             = 0x2D,   //            r
	L3GD20_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
	L3GD20_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
	L3GD20_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
	L3GD20_REGISTER_INT1_SRC            = 0x31,   //            r
	L3GD20_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
	L3GD20_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
	L3GD20_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
	L3GD20_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
	L3GD20_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
	L3GD20_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
	L3GD20_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
} l3gd20Registers_t;

void setup(){
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	Serial.begin(9600);
	Wire.begin();

	delay(2000);
	Serial.println("Reqesting ID from I2C");
	Wire.beginTransmission(L3GD20_ADDRESS);
	Wire.write((byte)L3GD20_REGISTER_WHO_AM_I);
	Wire.endTransmission();
	Wire.requestFrom((byte)L3GD20_ADDRESS, (byte)1);
	byte value = Wire.read();
	Wire.endTransmission();
	uint8_t id = value;
	Serial.print("ID:"); Serial.print((byte)value, HEX);
	if ((id != L3GD20_ID) && (id != L3GD20H_ID))
	{
		Serial.println("STOP. NOT CORRECT DEVICE");
	}
	else{
		
	/* Set CTRL_REG1 (0x20)
	 ====================================================================
	 BIT  Symbol    Description                                   Default
	 ---  ------    --------------------------------------------- -------
	 7-6  DR1/0     Output data rate                                   00
	 5-4  BW1/0     Bandwidth selection                                00
		 3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
		 2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
		 1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
		 0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */
		Wire.beginTransmission(L3GD20_ADDRESS);
		Wire.write((byte)L3GD20_REGISTER_CTRL_REG1);
		Wire.write(0x0F);
		Wire.endTransmission();
	
	/* Set CTRL_REG4 (0x23)
	 ====================================================================
	 BIT  Symbol    Description                                   Default
	 ---  ------    --------------------------------------------- -------
		 7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
		 6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
	 5-4  FS1/0     Full scale selection                               00
																	00 = 250 dps
																	01 = 500 dps
																	10 = 2000 dps
																	11 = 2000 dps
		 0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */
		Wire.beginTransmission(L3GD20_ADDRESS);
		Wire.write((byte)L3GD20_REGISTER_CTRL_REG4);
		Wire.write(0x00);
		Wire.endTransmission();
	}
}

float accelX, accelY, accelZ;

#define VOLTS 3.0

// smoothing, avg polls over interval
#define INTERVAL 10 // ms between averaging
unsigned int smoothingDivider;
float cumulativeX, cumulativeY, cumulativeZ;
float smoothX, smoothY, smoothZ;

void loop(){
//	int a0 = analogRead(A0);
//	int a1 = analogRead(A1);
//	int a2 = analogRead(A2);
//	accelX = (a0 - 321.5) / 62.5;// / VOLTS / 255.0;  
//	accelY = (a1 - 326.5) / 64.0;// / VOLTS / 255.0;
//	accelZ = (a2 - 330.5) / 64.5;// / VOLTS / 255.0;

	// READ L3GD20 GYROSCOPE ON I2C
	Wire.beginTransmission(L3GD20_ADDRESS);
	// Make sure to set address auto-increment bit
	Wire.write(L3GD20_REGISTER_OUT_X_L | 0x80);
	Wire.endTransmission();
	Wire.requestFrom((byte)L3GD20_ADDRESS, (byte)6);
	// Wait around until enough data is available
	while (Wire.available() < 6);   
	uint8_t xhi, xlo, ylo, yhi, zlo, zhi;
	xlo = Wire.read();
	xhi = Wire.read();
	ylo = Wire.read();
	yhi = Wire.read();
	zlo = Wire.read();
	zhi = Wire.read();

	float sampleX = (int)(xlo | (xhi << 8));
	float sampleY = (int)(ylo | (yhi << 8));
	float sampleZ = (int)(zlo | (zhi << 8));
	sampleX *= L3GD20_SCALE * DEG_RAD;  // * 0.125
	sampleY *= L3GD20_SCALE * DEG_RAD;  // * 0.125
	sampleZ *= L3GD20_SCALE * DEG_RAD;  // * 0.125
	
	// sample smoothing
	cumulativeX += sampleX;
	cumulativeY += sampleY;
	cumulativeZ += sampleZ;
	smoothingDivider++;

	static unsigned long smoothMillisPoll = 0;
	if(millis() > smoothMillisPoll + INTERVAL){
		smoothMillisPoll = millis();

		smoothX = cumulativeX/smoothingDivider;
		smoothY = cumulativeY/smoothingDivider;
		smoothZ = cumulativeZ/smoothingDivider;
		smoothingDivider = cumulativeX = cumulativeY = cumulativeZ = 0;
	}


	if(abs(sampleX) > .1)
		digitalWrite(13,HIGH);
	else
		digitalWrite(13,LOW);

	// accel (XYZ), gyro (XYZ)
	// MadgwickAHRSupdateIMU(xAccel, yAccel, zAccel, smoothX, smoothY, smoothZ);
//	MadgwickAHRSupdateIMU(sampleX, sampleY, sampleZ, accelX, accelY, accelZ);

//	static unsigned int logCount = 0;
//	logCount++;
//	if(logCount >= 20){
//		logCount = 0;
//		Serial.print("(");	Serial.print(sampleFreq);  
//		Serial.print(")Hz\t(");	Serial.print(smoothX); 
//		Serial.print(", ");	Serial.print(smoothY);
//		Serial.print(", ");	Serial.print(smoothZ);	Serial.print(")");
//		Serial.print("\t\t(");	Serial.print(accelX); 
//		Serial.print(", ");	Serial.print(accelY);
//		Serial.print(", ");	Serial.print(accelZ);	Serial.print(")");
//		Serial.print("\t\tQ(");	Serial.print(q0);  
//		Serial.print(", ");	Serial.print(q1); 
//		Serial.print(", ");	Serial.print(q2);
//		Serial.print(", ");	Serial.print(q3);	Serial.println(")"); 
//	}
}
