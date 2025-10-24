#include <Wire.h>
#include <Arduino_LSM6DS3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;  // kept for compatibility, not used

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB);
  SerialUSB.println("SAMD21 XBee + LSM6DS3 (Simplified: Downward Accel + Rotation) Starting...");

  Serial1.begin(9600);  // XBee communication
  SerialUSB.println("Serial1 (XBee) active at 9600 baud.");

  if (!IMU.begin()) {
    SerialUSB.println("Failed to detect LSM6DS3 sensor!");
    while (1);
  }
  SerialUSB.println("LSM6DS3 detected successfully!");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable())    IMU.readGyroscope(gx, gy, gz);

  // Compute rotation magnitude (single rotation value)
  float rotation = sqrt(gx * gx + gy * gy + gz * gz);

  // ---- Transmit two values ----
  // Format: az,rotation
  Serial1.print(az, 3);
  Serial1.print(",");
  Serial1.println(rotation, 3);

  // ---- Check for incoming data from XBee ----
  while (Serial1.available()) {
    char c = Serial1.read();
    SerialUSB.write(c);
  }

  // ---- Optional local debugging ----
  SerialUSB.print("Downward Accel (Z): ");
  SerialUSB.print(az, 3);
  SerialUSB.print(" m/s^2 | Rotation Magnitude: ");
  SerialUSB.print(rotation, 3);
  SerialUSB.println(" °/s");

  delay(200);
}
