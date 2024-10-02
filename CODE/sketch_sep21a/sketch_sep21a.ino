#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MPU6050 object
Adafruit_MPU6050 mpu;

// Define PID parameters
float Kp = 5;
float Ki = 0;
float Kd = 0;

// Define error variables for PID
float error_pitch, previous_error_pitch = 0;
float error_roll, previous_error_roll = 0;
float integral_pitch = 0, integral_roll = 0;
float derivative_pitch, derivative_roll;

// Time variables
unsigned long current_time, previous_time;
float elapsed_time;

// Stepper motor control pins for ESP8266
int D0 = 16, D1 = 5, D2 = 4, D3 = 0, D4 = 2, D5 = 14, D6 = 13, D7 = 12;
const int dirPinLeft = D1;      // GPIO 5
const int stepPinLeft = D2;     // GPIO 4
const int dirPinRight = D5;     // GPIO 14
const int stepPinRight = D0;    // GPIO 16

// Define motor steps per revolution
const int stepsPerRevolution = 200;

// Motor speed variables (adjustable)
int baseSpeed = 1000; // Delay in microseconds for normal speed

// Define debug pin
const int debugPin = 1; // GPIO1 (TX PIN)

// Variable to store debug mode state
bool debugMode = false;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // wait for serial connection

  // Initialize debug pin
  pinMode(debugPin, INPUT_PULLUP); // Assuming debug is active LOW

  // Initial read to set debugMode
  debugMode = (digitalRead(debugPin) == LOW);

  if (debugMode)
    Serial.println("Adafruit MPU6050 test!");

  // Initialize MPU6050
  Wire.begin(D6, D7);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  if (debugMode)
    Serial.println("MPU6050 Found!");

  // Set accelerometer and gyro ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Initialize motor pins as output
  pinMode(stepPinLeft, OUTPUT);
  pinMode(dirPinLeft, OUTPUT);
  pinMode(stepPinRight, OUTPUT);
  pinMode(dirPinRight, OUTPUT);

  // Initialize time variables
  previous_time = millis();
}

void loop() {
  // Update debugMode based on the debug pin state
  debugMode = (digitalRead(debugPin) == LOW); // Adjust logic if active HIGH

  // Get new sensor readings
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Calculate tilt angles
  float pitch_angle = atan2(accel.acceleration.x, accel.acceleration.z) * 180 / PI;
  pitch_angle += pitch_angle < 0 ? 360 : 0;

  // float roll_angle = atan2(accel.acceleration.x, accel.acceleration.z) * 180 / PI;

  // Call PID control for both pitch and roll
  int motor_output_left = 200;
  // int motor_output_right = PIDControl(pitch_angle, gyro.gyro.y);
  // motor_output_left = 150;
  int motor_output_right = motor_output_left;

  // Control the motors based on the PID output
  controlMotors(motor_output_left, motor_output_right);

  // Print the sensor data for debugging
  if (debugMode) {
    Serial.print("Acceleration X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(accel.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(", Y: ");
    Serial.print(gyro.gyro.y);
    Serial.print(", Z: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Pitch Angle: ");
    Serial.print(pitch_angle);
    Serial.print(", Roll Angle: ");
    // Serial.print(roll_angle);
    // Serial.println(" degrees");

    Serial.println("");
  }

  delay(100); // Adjust delay as needed
}

// PID control function
int PIDControl(float angle, float gyro_rate) {
  // Time calculations for derivative and integral
  current_time = millis();
  elapsed_time = (current_time - previous_time) / 1000.0; // Convert to seconds

  // Pitch PID control
  float error = 180 - angle; // Target angle is 0 (upright)
  integral_pitch += error * elapsed_time;
  derivative_pitch = (error - previous_error_pitch) / elapsed_time;

  // Calculate control output
  float output = Kp * error + Ki * integral_pitch + Kd * derivative_pitch;

  // Save errors for next iteration
  previous_error_pitch = error;
  previous_time = current_time;

  return constrain(output, -stepsPerRevolution, stepsPerRevolution); // Constrain output to stepper motor step limits
}

// Function to control motors
void controlMotors(int motor_output_left, int motor_output_right) {
  // Adjust speed and direction based on PID output
  int motorSpeedLeft = 1000 + baseSpeed - abs(motor_output_left) * 5;
  int motorSpeedRight = 1000 + baseSpeed - abs(motor_output_right) * 5;

  if (debugMode) {
    Serial.println("motor delay");
    Serial.println(motorSpeedLeft);
  }

  // Set motor direction
  digitalWrite(dirPinLeft, motor_output_left > 0 ? HIGH : LOW);
  digitalWrite(dirPinRight, motor_output_right > 0 ? HIGH : LOW);

  int xx = 0;

  // Spin left motor
  for (int x = 0; x < abs(motor_output_left) - xx; x++) {
    digitalWrite(stepPinLeft, HIGH);
    delayMicroseconds(motorSpeedLeft);
    digitalWrite(stepPinLeft, LOW);
    delayMicroseconds(motorSpeedLeft);
  }

  // Spin right motor
  for (int x = 0; x < abs(motor_output_right) - xx; x++) {
    digitalWrite(stepPinRight, HIGH);
    delayMicroseconds(motorSpeedRight);
    digitalWrite(stepPinRight, LOW);
    delayMicroseconds(motorSpeedRight);
  }
}
