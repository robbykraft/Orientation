# orientation

### orientation awareness on minimal hardware, minimal cost, open source

Bluetooth-low energy communicating quaternion orientation, raw accelerometer data, and any extra sensors (flex sensors for glove).

# software

* Xcode project receives Bluetooth LE signal, outputs OSC over localhost
* Puredata patch receives OSC signals

# hardware

* Arduino (Atmel AVR) *or* Teensy 3.X (ARM Cortex)
* gyroscope L3GD20H
* accelerometer + magnetometer LSM303
* Adafruit nRF8001 Bluetooth LE

# thanks

Sebastian Madgwick's IMU and AHRS functions
Adafruit nRF8001 BLE library
Paul Stoffregen's 32bit ARM updates