#include <Wire.h>
#include <Arduino_LSM6DS3.h>   // For LSM6DS3 IMU
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>   // (kept for consistency if needed)

Adafruit_BMP3XX bmp;  // unused but retained for compatibility

void setup() {
  // Initialize SerialUSB for debugging
  SerialUSB.begin(115200);
  while (!SerialUSB);
  SerialUSB.println("SAMD21 XBee + LSM6DS3 Unified System Starting...");

  // Initialize Serial1 for XBee communication
  Serial1.begin(9600);
  SerialUSB.println("Serial1 (XBee) active at 9600 baud.");

  // Initialize the LSM6DS3 IMU
  if (!IMU.begin()) {
    SerialUSB.println("Failed to detect LSM6DS3 sensor!");
    while (1);
  }
  SerialUSB.println("LSM6DS3 detected successfully!");
}

void loop() {
  // ---- Read sensor data ----
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable())    IMU.readGyroscope(gx, gy, gz);

  // ---- Format and transmit data ----
  // Comma-separated: ax,ay,az,gx,gy,gz
  Serial1.print(ax, 3); Serial1.print(",");
  Serial1.print(ay, 3); Serial1.print(",");
  Serial1.print(az, 3); Serial1.print(",");
  Serial1.print(gx, 3); Serial1.print(",");
  Serial1.print(gy, 3); Serial1.print(",");
  Serial1.println(gz, 3);

  // ---- Check for incoming XBee data ----
  while (Serial1.available()) {
    char c = Serial1.read();
    SerialUSB.write(c);  // Echo received data to USB serial monitor
  }

  // ---- Optional: also print locally ----
  SerialUSB.print("Accel: ");
  SerialUSB.print(ax, 3); SerialUSB.print(", ");
  SerialUSB.print(ay, 3); SerialUSB.print(", ");
  SerialUSB.println(az, 3);

  SerialUSB.print("Gyro: ");
  SerialUSB.print(gx, 3); SerialUSB.print(", ");
  SerialUSB.print(gy, 3); SerialUSB.print(", ");
  SerialUSB.println(gz, 3);

  delay(200); // sample rate ~5Hz
}
