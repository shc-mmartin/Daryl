#include <Wire.h>
#include <Arduino_LSM6DS3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;  // retained for compatibility, unused

// Velocity integration variables
float velocity = 0.0;      // vertical velocity (m/s)
unsigned long lastUpdate = 0;

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB);
  SerialUSB.println("SAMD21 XBee + LSM6DS3 (Velocity + Rotation) Starting...");

  Serial1.begin(9600);  // XBee communication
  SerialUSB.println("Serial1 (XBee) active at 9600 baud.");

  if (!IMU.begin()) {
    SerialUSB.println("Failed to detect LSM6DS3 sensor!");
    while (1);
  }
  SerialUSB.println("LSM6DS3 detected successfully!");

  lastUpdate = millis();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable())    IMU.readGyroscope(gx, gy, gz);

  // Compute time difference (seconds)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdate) / 1000.0;
  lastUpdate = currentTime;

  // Adjust acceleration for gravity (assuming Z is vertical)
  // Integrate acceleration to get velocity
  velocity += az * dt;

  // Compute rotation magnitude
  float rotation = sqrt(gx * gx + gy * gy + gz * gz);

  // ---- Transmit two values ----
  // Format: velocity,rotation
  Serial1.print(velocity, 3);
  Serial1.print(",");
  Serial1.println(rotation, 3);

  // ---- Check for incoming XBee data ----
  while (Serial1.available()) {
    char c = Serial1.read();
    SerialUSB.write(c);
  }

  // ---- Optional local debug output ----
  SerialUSB.print("Vertical Velocity: ");
  SerialUSB.print(velocity, 3);
  SerialUSB.print(" m/s | Rotation Magnitude: ");
  SerialUSB.print(rotation, 3);
  SerialUSB.println(" °/s");

  delay(50);  // sample at 20 Hz
}
