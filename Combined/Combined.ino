#include <Wire.h>
#include <Arduino_LSM6DS3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;  // retained for compatibility, unused

// Velocity integration variables
float velocity = 0.0;         // vertical velocity (m/s)
unsigned long lastUpdate = 0;
float gravityRef = 9.81;      // measured gravity reference

// Calibration settings
const int CALIBRATION_SAMPLES = 200;  // how many samples to average
const float DRIFT_DECAY = 0.98;       // how fast velocity decays toward 0 when idle
const float MOTION_THRESHOLD = 0.1;   // m/s² threshold for "no movement"
const float ROTATION_THRESHOLD = 1.0; // °/s threshold for "no rotation"

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB);
  SerialUSB.println("SAMD21 XBee + LSM6DS3 (Velocity + Rotation + Drift Fix) Starting...");

  Serial1.begin(9600);  // XBee communication
  SerialUSB.println("Serial1 (XBee) active at 9600 baud.");

  if (!IMU.begin()) {
    SerialUSB.println("Failed to detect LSM6DS3 sensor!");
    while (1);
  }
  SerialUSB.println("LSM6DS3 detected successfully!");

  // --- Calibrate gravity reference ---
  SerialUSB.println("Calibrating gravity reference... keep still.");
  float sumAz = 0;
  int samples = 0;
  while (samples < CALIBRATION_SAMPLES) {
    float ax, ay, az;
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      sumAz += az;
      samples++;
      delay(5);
    }
  }
  gravityRef = sumAz / samples;
  SerialUSB.print("Calibrated gravity: ");
  SerialUSB.println(gravityRef, 4);

  lastUpdate = millis();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable())    IMU.readGyroscope(gx, gy, gz);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdate) / 1000.0;
  lastUpdate = currentTime;

  // Calculate net acceleration (remove calibrated gravity)
  float netAz = az - gravityRef;

  // Compute rotation magnitude
  float rotation = sqrt(gx * gx + gy * gy + gz * gz);

  // Integrate acceleration to velocity
  velocity += netAz * dt;

  // ---- Drift correction ----
  // If the board is nearly still (no acceleration & rotation), decay velocity toward 0
  if (fabs(netAz) < MOTION_THRESHOLD && rotation < ROTATION_THRESHOLD) {
    velocity *= DRIFT_DECAY;
  }

  // ---- Transmit two values ----
  // Format: velocity,rotation
  Serial1.print(velocity, 3);
  Serial1.print(",");
  Serial1.println(rotation, 3);

  // ---- Check for incoming data from XBee ----
  while (Serial1.available()) {
    char c = Serial1.read();
    SerialUSB.write(c);
  }

  // ---- Optional debug ----
  SerialUSB.print("Vertical Velocity: ");
  SerialUSB.print(velocity, 3);
  SerialUSB.print(" m/s | Rotation: ");
  SerialUSB.print(rotation, 3);
  SerialUSB.print(" °/s | netAz: ");
  SerialUSB.println(netAz, 3);

  delay(50);  // 20 Hz
}
