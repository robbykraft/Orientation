#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Wire.h> 
#include <Adafruit_L3GD20.h>
#include <Adafruit_LSM303.h>

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

Adafruit_LSM303 lsm;
Adafruit_L3GD20 gyro;

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;


#define DEG_RAD 0.01745329251994
  
  
void setup(void)
{ 
  
//  pinMode(13,OUTPUT);
  Serial.begin(9600);
  while(!Serial);
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  // BTLEserial.setDeviceName("NEWNAME"); /* 7 characters max! */

  BTLEserial.begin();
  
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }

}

#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.5f		// half the sample period

//---------------------------------------------------------------------------------------------------
// Variable definitions

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	// quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error

//====================================================================================================
// Function
//====================================================================================================

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;

	// auxiliary variables to reduce number of repeated operations
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;          
	
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);       
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;
	norm = sqrt(mx*mx + my*my + mz*mz);          
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;         
	
	// compute reference direction of flux
	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;        
	
	// estimated direction of gravity and flux (v and w)
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
}

void loop()
{
  lsm.read();
  gyro.read();
  
  AHRSupdate(gyro.data.x * DEG_RAD, gyro.data.y * DEG_RAD, gyro.data.z * DEG_RAD, 
             lsm.accelData.x * DEG_RAD, lsm.accelData.y * DEG_RAD, lsm.accelData.z * DEG_RAD, 
             lsm.magData.x * DEG_RAD, lsm.magData.y * DEG_RAD, lsm.magData.z * DEG_RAD);

  Serial.print("-----  ");   Serial.print("X\t");
  Serial.print("");          Serial.print("Y\t");
  Serial.print("");          Serial.println("Z");

  Serial.print("ACCEL: "); Serial.print((int)lsm.accelData.x);  Serial.print("\t");
  Serial.print(""); Serial.print((int)lsm.accelData.y);  Serial.print("\t");
  Serial.print(""); Serial.print((int)lsm.accelData.z);  Serial.println("");

  Serial.print("MAGNO: ");    Serial.print((int)lsm.magData.x);    Serial.print("\t");
  Serial.print("");    Serial.print((int)lsm.magData.y);    Serial.print("\t");
  Serial.print("");    Serial.print((int)lsm.magData.z);    Serial.println("");  

  Serial.print("GYROS: ");    Serial.print((int)gyro.data.x);      Serial.print("\t");
  Serial.print("");    Serial.print((int)gyro.data.y);      Serial.print("\t");
  Serial.print("");    Serial.print((int)gyro.data.z);      Serial.println("");
  
  Serial.print("QUAT: "); Serial.print(q0);      Serial.print("\t");
  Serial.print("");    Serial.print(q1);      Serial.print("\t");
  Serial.print("");    Serial.print(q2);      Serial.print("\t");
  Serial.print("");    Serial.print(q3);      Serial.println("");

  delay(1000);
//  digitalWrite(13,HIGH);
//  delay(50);
//  digitalWrite(13,LOW);

  
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  aci_evt_opcode_t BLEStatus = BTLEserial.getState();
//  ACI_EVT_INVALID                     = 0
//  ACI_EVT_DEVICE_STARTED              = 129
//  ACI_EVT_ECHO                        = 130
//  ACI_EVT_HW_ERROR                    = 131
//  ACI_EVT_CMD_RSP                     = 132
//  ACI_EVT_CONNECTED                   = 133
//  ACI_EVT_DISCONNECTED                = 134
//  ACI_EVT_BOND_STATUS                 = 135
//  ACI_EVT_PIPE_STATUS                 = 136
//  ACI_EVT_TIMING                      = 137
//  ACI_EVT_DATA_CREDIT                 = 138
//  ACI_EVT_DATA_ACK                    = 139
//  ACI_EVT_DATA_RECEIVED               = 140
//  ACI_EVT_PIPE_ERROR                  = 141
//  ACI_EVT_DISPLAY_PASSKEY             = 142
//  ACI_EVT_KEY_REQUEST                 = 143
  
  
  Serial.println(BLEStatus);
  // If the status changed....
  if (BLEStatus != laststatus) {
    // print it out!
    if (BLEStatus == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (BLEStatus == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (BLEStatus == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = BLEStatus;
  }

  if (BLEStatus == ACI_EVT_CONNECTED) {    
        
    uint8_t gyroBuffer[2];
//    gyroBuffer[0] = xhi;
//    gyroBuffer[1] = xlo;
    char bufferSize = 2;
    
    int gyroX = (int)q1;
    char gyroXChar = (char)gyroX;
    gyroBuffer[0] = gyroXChar;
    
    gyroX = (int)gyro.data.x;
    gyroXChar = (char)gyroX;
    gyroBuffer[1] = gyroXChar;

    BTLEserial.write(gyroBuffer, bufferSize);


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
