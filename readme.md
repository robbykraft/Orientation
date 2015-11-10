# orientation

### orientation awareness, minimal hardware, minimal cost, open source

1. Bluetooth-low energy communicates quaternion orientation, raw accelerometer data, any extra sensors (flex sensors for glove)
2. Mac app converts BLE data into OSC commands over localhost for use with Puredata, MaxMSP, Ableton...

# software

* Mac OS X app, converts BLE to OSC
* Puredata example patch

# hardware

* Arduino (Atmel AVR) *or* Teensy 3.X (ARM Cortex)
* gyroscope L3GD20H
* accelerometer + magnetometer LSM303
* Adafruit nRF8001 Bluetooth LE

# thanks

* Sebastian Madgwick's IMU and AHRS functions
* Adafruit nRF8001 BLE library
* Paul Stoffregen's 32bit ARM updates
* Ross Bencina and OSCPack