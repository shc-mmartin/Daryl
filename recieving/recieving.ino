#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;


void setup() {
  // SerialUSB for debugging
  SerialUSB.begin(115200);
  while (!SerialUSB);
  SerialUSB.println("XBee Receiver Starting...");

  // Initialize Serial1 for XBee
  Serial1.begin(9600);  // Match sender’s baud rate
  SerialUSB.println("Serial1 (XBee) listening at 9600 baud...");
}

void loop() {
  // Check if XBee has sent any data
  while (Serial1.available()) {
    char c = Serial1.read();     // Read one byte
    SerialUSB.write(c);          // Echo it to the Serial Monitor
  }
}


