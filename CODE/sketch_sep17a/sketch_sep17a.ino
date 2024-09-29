#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(115200);

    // Initialize I2C communication and MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    Serial.println("MPU6050 Found!");
  
    // Configure the sensor ranges (optional)
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Set accelerometer to ±8g
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // Set gyroscope to ±500°/s
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Set bandwidth to 21Hz

    delay(100); // Wait for sensor stabilization
}

void loop() {
    // Get new sensor events with the sensor_t type
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate pitch angle based on accelerometer readings
    float pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

    // Print out the pitch angle
    Serial.print("Pitch: ");
    Serial.println(pitch);
  
    delay(100);
}
