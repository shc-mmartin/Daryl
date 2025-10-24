#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;

void setup() {
  // Debug USB serial
  SerialUSB.begin(115200);
  while (!SerialUSB);
  SerialUSB.println("XBee Random Number Sender Starting...");

  // XBee serial port
  Serial1.begin(9600);  // TX/RX connected to XBee
  SerialUSB.println("Serial1 (XBee) initialized at 9600 baud.");

  // Seed random number generator (optional, using floating ADC pin for entropy)
  randomSeed(analogRead(A0));
}

void loop() {
  // Generate a random number between 0 and 9999
  int randomValue = random(0, 10000);

  // Print locally
  SerialUSB.print("Sending: ");
  SerialUSB.println(randomValue);

  // Send through XBee
  Serial1.println(randomValue);

  delay(2000); // Wait 2 seconds
}

