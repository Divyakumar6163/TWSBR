#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AccelStepper.h>
// #include <ESP8266WiFi.h>
// #include <ArduinoOTA.h>
// MPU6050 object
Adafruit_MPU6050 mpu;

// Define PID parameters
float Kp = 4.9;
float Kd = 0;
float Ki = 0;

// Define error variables for PID
float error_pitch, previous_error_pitch = 0;
float integral_pitch = 0;
float derivative_pitch;

// Time variables
unsigned long current_time, previous_time;
float elapsed_time;

// Stepper motor control pins for ESP8266
int D0 = 16, D1 = 5, D2 = 4, D3 = 0, D4 = 2, D5 = 14, D6 = 13, D7 = 12;

// Define stepper motor interface and pins
const int dirPinLeft = D1;    // GPIO 5
const int stepPinLeft = D2;   // GPIO 4
const int dirPinRight = D5;   // GPIO for the second motor's direction
const int stepPinRight = D0;  // GPIO for the second motor's steps

// Define AccelStepper motor objects (using DRIVER interface)
AccelStepper stepperLeft(AccelStepper::DRIVER, stepPinLeft, dirPinLeft);
AccelStepper stepperRight(AccelStepper::DRIVER, stepPinRight, dirPinRight);

// Define motor steps per revolution
const int stepsPerRevolution = 200;

// const char* ssid = "realme C1221";
// const char* password = "123456789888";

// Setup function
void setup() {
  Serial.begin(19200);
  // while (!Serial) delay(10); // wait for serial connection
  Serial.println("Adafruit MPU6050 test!");

  // Initialize MPU6050
  Wire.begin(D6, D7);  // Use D6 and D7 as SDA and SCL
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    // while (1) delay(10);
  }
  // mpu.begin();
  Serial.println("MPU6050 Found!");

  // MPU settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // myStepper.setMaxSpeed(1000);
  // myStepper.setAcceleration(50);
  // myStepper.setSpeed(200);
  // myStepper.moveTo(200);
  // Initialize motors
  stepperLeft.setMaxSpeed(1000);     // Set maximum speed in steps/second
  stepperLeft.setAcceleration(600);  // Set acceleration in steps/second^2
  stepperLeft.setSpeed(800);

  stepperRight.setMaxSpeed(1000);  // Same for the right motor
  stepperRight.setAcceleration(600);
  stepperRight.setSpeed(800);
  // Initialize time variables
  previous_time = millis();


  //  // Connect to Wi-Fi
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }
  // Serial.println("Connected to WiFi");

  // // OTA Setup
  // ArduinoOTA.onStart([]() {
  //   String type;
  //   if (ArduinoOTA.getCommand() == U_FLASH) {
  //     type = "sketch";
  //   } else { // U_SPIFFS
  //     type = "filesystem";
  //   }
  //   // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  //   Serial.println("Start updating " + type);
  // });

  // ArduinoOTA.onEnd([]() {
  //   Serial.println("\nEnd");
  // });

  //  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  //   Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  // });

  // ArduinoOTA.onError([](ota_error_t error) {
  //   Serial.printf("Error[%u]: ", error);
  //   if (error == OTA_AUTH_ERROR) {
  //     Serial.println("Auth Failed");
  //   } else if (error == OTA_BEGIN_ERROR) {
  //     Serial.println("Begin Failed");
  //   } else if (error == OTA_CONNECT_ERROR) {
  //     Serial.println("Connect Failed");
  //   } else if (error == OTA_RECEIVE_ERROR) {
  //     Serial.println("Receive Failed");
  //   } else if (error == OTA_END_ERROR) {
  //     Serial.println("End Failed");
  //   }
  // });

  // ArduinoOTA.begin();
  // Serial.println("OTA Ready");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());
  delay(500);
}

// PID control function
int targetAngle = 179;
int PIDControl(float angle, float gyro_rate) {
  // Time calculations for derivative and integral
  current_time = millis();
  elapsed_time = (current_time - previous_time) / 1000.0;  // Convert to seconds

  // Pitch PID control
  float error = targetAngle - angle;  // Target angle is 180 degrees (upright)
  
  integral_pitch += error * elapsed_time;
  derivative_pitch = (error - previous_error_pitch) / elapsed_time;

  // Calculate control output
  float output = Kp * error + Ki * integral_pitch + Kd * derivative_pitch;

  // Save errors for next iteration
  previous_error_pitch = error;
  previous_time = current_time;

  return constrain(output, -500, 500);  // Constrain output for stepper step range
}

// Function to control motors using AccelStepper
void controlMotors(int motor_output_left, int motor_output_right) {
  if(motor_output_left> 550 || motor_output_left<-550 || motor_output_left==0) return;
  // Serial.print("Steps:   ");
  // Serial.println(motor_output_left);
  // Set the speed proportional to the PID output
  // motor_output_left=200;
  // motor_output_right=200;
  // stepperLeft.setSpeed(motor_output_left);  // Set speed for the left motor
  // stepperRight.setSpeed(motor_output_right);  // Set speed for the right motor

  // Move the motors
//   for(int i=0;i<motor_output_left;i++){
//     stepperLeft.runSpeed();  // Run the left motor at the set speed
//     stepperRight.runSpeed(); // Run the right motor at the set speed
//   }
// delay(100);

  stepperLeft.move(-motor_output_left);
  stepperRight.move(-motor_output_right);
  while (stepperLeft.distanceToGo() != 0) {
    stepperLeft.run();
    stepperRight.run();
  }
}

void loop() {
  // ArduinoOTA.handle(); 
  // Get new sensor readings
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Calculate tilt angles
  float pitch_angle = atan2(accel.acceleration.x, accel.acceleration.z) * 180 / PI;
  pitch_angle += pitch_angle < 0 ? 360 : 0;  // Adjust pitch angle

  // Call PID control for pitch
  int motor_output_left = PIDControl(pitch_angle, gyro.gyro.x);
  int motor_output_right = motor_output_left;  // Assuming both motors are balanced for pitch
  // Serial.println(motor_output_right);
  // Control the motors based on the PID output

  // unsigned long startTime = millis();  // Record start time
  controlMotors(motor_output_left, motor_output_right);
  // unsigned long endTime = millis();    // Record end time
  // unsigned long duration = endTime - startTime;  // Calculate duration
  

  // Serial.print("Function took ");
  // Serial.print(duration);
  // Serial.println(" ms to execute.");

  // // Print the sensor data for debugging
  Serial.print("Pitch Angle: ");
  Serial.println(pitch_angle);
  // Serial.println(" degrees");

  // delay(100); // Adjust delay as needed
  yield(); // Allow ESP8266 to handle background tasks
}