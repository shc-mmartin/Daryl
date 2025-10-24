#include <Arduino_LSM6DS3.h>

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB);

  SerialUSB.println("LSM6DS3 test");

  if (!IMU.begin()) {  // auto-detects address (0x6A or 0x6B)
    SerialUSB.println("Failed to detect LSM6DS3 sensor!");
    while (1);
  }

  SerialUSB.println("LSM6DS3 detected!");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable())    IMU.readGyroscope(gx, gy, gz);

  SerialUSB.print("Accel: "); SerialUSB.print(ax); SerialUSB.print(", "); SerialUSB.print(ay); SerialUSB.print(", "); SerialUSB.println(az);
  SerialUSB.print("Gyro:  ");  SerialUSB.print(gx); SerialUSB.print(", "); SerialUSB.print(gy); SerialUSB.print(", "); SerialUSB.println(gz);

  delay(200);
}
