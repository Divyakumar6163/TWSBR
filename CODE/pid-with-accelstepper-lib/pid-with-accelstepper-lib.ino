#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AccelStepper.h>

Adafruit_MPU6050 mpu;

// PID parameters
float Kp = 80;
float Kd = 0;
float Ki = 0;

long int motor_output;
float pitch_angle;

// PID variables
float error_pitch, previous_error_pitch = 0;
float integral_pitch = 0;
float derivative_pitch;
unsigned long current_time, previous_time;
float elapsed_time;

float Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float rad_to_deg = 180 / 3.141592654;

// Stepper motor pins for ESP8266
int D0 = 16, D1 = 5, D2 = 4, D3 = 0, D4 = 2, D5 = 14, D6 = 13, D7 = 12;

const int dirPinLeft = D1;
const int stepPinLeft = D2;
const int dirPinRight = D5;
const int stepPinRight = D0;

// AccelStepper objects (using DRIVER interface)
AccelStepper stepperLeft(AccelStepper::DRIVER, stepPinLeft, dirPinLeft);
AccelStepper stepperRight(AccelStepper::DRIVER, stepPinRight, dirPinRight);

const int maxMotorSpeed = 2500; // Max motor speed in steps/sec
const int maxMotorAccel = 1200; // Max motor acceleration in steps/sec^2

void setup()
{
  Serial.begin(9600);
  Serial.println("Adafruit MPU6050 test!");

  Wire.begin(D6, D7); // Use D6 and D7 as SCL and SDA

  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip, retrying...");
    while (!mpu.begin())
    {
      delay(100);
    }
  }
  Serial.println("MPU6050 Found!");

  // // MPU settings
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize motors
  stepperLeft.setMaxSpeed(maxMotorSpeed);     // Max speed in steps/second
  stepperLeft.setAcceleration(maxMotorAccel); // Acceleration in steps/second^2
  stepperRight.setMaxSpeed(maxMotorSpeed);
  stepperRight.setAcceleration(maxMotorAccel);
  delay(100);
  current_time = millis();
}

float targetAngle = 0;
int PIDControl(float angle)
{
  float error = targetAngle - angle;
  integral_pitch += error * elapsed_time;
  derivative_pitch = (error - previous_error_pitch) / elapsed_time;

  float output = Kp * error + Ki * integral_pitch + Kd * derivative_pitch;

  previous_error_pitch = error;
  return constrain(output, -maxMotorSpeed, maxMotorSpeed);
}

void controlMotors(int motor_output_left, int motor_output_right)
{
  // stepperLeft.move(-motor_output_left);
  // stepperRight.move(+motor_output_right);

  stepperLeft.setSpeed(-motor_output_left);
  stepperRight.setSpeed(motor_output_right);
}

void loop()
{
  previous_time = current_time;
  current_time = millis();
  elapsed_time = (current_time - previous_time) / 1000.0;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Acc_rawX = accel.acceleration.x;
  Acc_rawY = accel.acceleration.y;
  Acc_rawZ = accel.acceleration.z;
  Acceleration_angle[0] = atan((Acc_rawY) / (sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2)) == 0 ? 1 : sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2)))) * rad_to_deg;
  Acceleration_angle[1] = atan(-1 * (Acc_rawX) / (sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2)) == 0 ? 1 : sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2)))) * rad_to_deg;

  Gyr_rawX = gyro.gyro.x;
  Gyr_rawY = gyro.gyro.y;

  Gyro_angle[0] = Gyr_rawX;
  Gyro_angle[1] = Gyr_rawY;

  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsed_time) + 0.02 * Acceleration_angle[0];
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsed_time) + 0.02 * Acceleration_angle[1];

  // Serial.println("*BEGIN*");
  // Serial.print("Acceleration X ");
  // Serial.println(Acc_rawX);
  // Serial.print("Acceleration Y ");
  // Serial.println(Acc_rawY);
  // Serial.print("Acceleration Y ");
  // Serial.println(Acc_rawY);

  // Serial.print("Gyro X ");
  // Serial.println(Gyr_rawX);
  // Serial.print("Gyro Y ");
  // Serial.println(Gyr_rawY);

  // Serial.print("Total Angle 0: ");
  // Serial.println(Total_angle[0]);
  // Serial.print("Total Angle 1: ");
  // Serial.println(Total_angle[1]);
  // Serial.println("*END*");
  // Serial.println();

  motor_output = PIDControl(Total_angle[1]);
  controlMotors(motor_output, motor_output);

  // if (stepperLeft.distanceToGo() != 0) {
  //   stepperLeft.run();
  // }
  // if (stepperRight.distanceToGo() != 0) {
  //   stepperRight.run();
  // }

  stepperLeft.runSpeed();
  stepperRight.runSpeed();

  //   Serial.print("Motor : ");
  // Serial.println(motor_output);
  // float currentSpeed = stepperRight.speed(); // Get the current speed in steps per second
  // Serial.print("Current speed: ");
  // Serial.println(currentSpeed);

  // if (isnan(Total_angle[1])) {
  //   mpu.begin();
  // }

  // // Serial.println(motor_output);
  yield();
}