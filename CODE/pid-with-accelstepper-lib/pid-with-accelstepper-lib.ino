#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AccelStepper.h>

Adafruit_MPU6050 mpu;

// PID parameters
float Kp = 3.5;
float Kd = 0;
float Ki = 0;

// PID variables
float error_pitch, previous_error_pitch = 0;
float integral_pitch = 0;
float derivative_pitch;
unsigned long current_time, previous_time;
float elapsed_time;

// Stepper motor pins for ESP8266
int D0 = 16, D1 = 5, D2 = 4, D3 = 0, D4 = 2, D5 = 14, D6 = 13, D7 = 12;

// Define stepper motor interface and pins
const int dirPinLeft = D1;    
const int stepPinLeft = D2;  
const int dirPinRight = D5;   
const int stepPinRight = D0;  

// AccelStepper objects (using DRIVER interface)
AccelStepper stepperLeft(AccelStepper::DRIVER, stepPinLeft, dirPinLeft);
AccelStepper stepperRight(AccelStepper::DRIVER, stepPinRight, dirPinRight);

// Motor control constants
// const int stepsPerRevolution = 00;
const int maxMotorSpeed = 2500;  // Max motor speed in steps/sec
const int maxMotorAccel = 1200;   // Max motor acceleration in steps/sec^2

void setup() {
  Serial.begin(19200);
  Serial.println("Adafruit MPU6050 test!");

  Wire.begin(D6, D7);  // Use D6 and D7 as SCL and SDA
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip, retrying...");
    while (!mpu.begin()) {
      delay(100);
    }
  }
  Serial.println("MPU6050 Found!");

  // MPU settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize motors
  stepperLeft.setMaxSpeed(maxMotorSpeed);  // Max speed in steps/second
  stepperLeft.setAcceleration(maxMotorAccel);  // Acceleration in steps/second^2
  stepperRight.setMaxSpeed(maxMotorSpeed);
  stepperRight.setAcceleration(maxMotorAccel);

  // Initialize time variables
  previous_time = millis();
}

// PID control function
float targetAngle = 179;
int PIDControl(float angle, float gyro_rate) {
  current_time = millis();
  elapsed_time = (current_time - previous_time) / 1000.0;  

  float error = targetAngle - angle;  // Target angle is 179 degrees (upright)
  integral_pitch += error * elapsed_time;
  derivative_pitch = (error - previous_error_pitch) / elapsed_time;

  float output = Kp * error + Ki * integral_pitch + Kd * derivative_pitch;
  

  previous_error_pitch = error;
  previous_time = current_time;

  return constrain(output, -maxMotorSpeed, maxMotorSpeed);
}


// void controlMotors(int motor_output_left, int motor_output_right) {
//   int speed = 200;
 
//     stepperLeft.move(motor_output_left);
//     stepperRight.move(-motor_output_right);
//     stepperLeft.setSpeed(speed);
//     stepperRight.setSpeed(speed);

//     while(stepperLeft.distanceToGo() != 0) {
//       stepperLeft.run();
//       stepperRight.run();
//     } 
// }

void controlMotors(int motor_output_left, int motor_output_right) {
  int speed = 200;

  // Set movement direction and steps
  stepperLeft.move(-motor_output_left);
  stepperRight.move(+motor_output_right);

  // Set speed
  // stepperLeft.setSpeed(speed);
  // stepperRight.setSpeed(speed);
}

long int prevtime=0;
long int motor_output;

float pitch_angle;


void loop() {
  // Serial.print(millis());
    // if(stepperLeft.distanceToGo() ==0 && stepperRight.distanceToGo()==0){
      sensors_event_t accel, gyro, temp;
      mpu.getEvent(&accel, &gyro, &temp);

      // Calculate pitch angle
      pitch_angle = atan2(accel.acceleration.x, accel.acceleration.z) * 180 / PI;
      pitch_angle += (pitch_angle < 0) ? 360 : 0;  // Adjust pitch angle


      // Serial.print("Pitch Angle: ");
      Serial.println(pitch_angle);
      // Call PID control for pitch
      motor_output = PIDControl(pitch_angle, gyro.gyro.x);

      prevtime=millis();
      // Serial.println(coutn);
      
      controlMotors(motor_output, motor_output);
  // }

  if (stepperLeft.distanceToGo() != 0) {
    stepperLeft.run();
  }

  if (stepperRight.distanceToGo() != 0) {
    stepperRight.run();
  }

   
  // Serial.println(motor_output);
  yield();  
} 