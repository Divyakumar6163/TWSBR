// Define pins for the ESP8266
const int dirPin = 4;  // GPIO 5
const int stepPin = 5; // GPIO 4
const int stepsPerRevolution = 1600;

void setup()
{
  // Declare pins as Outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop()
{
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);

  // Spin motor slowly
  for (int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(20); // 2ms delay
    digitalWrite(stepPin, LOW);
    delayMicroseconds(20); // 2ms delay
  }
  delay(1000); // Wait a second

  // Set motor direction counterclockwise
  digitalWrite(dirPin, LOW);

  // Spin motor quickly
  for (int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000); // 1ms delay
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000); // 1ms delay
  }
  delay(1000); // Wait a second

  digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level
  // but actually the LED is on; this is because
  // it is active low on the ESP-01)
  delay(1000);                     // Wait for a second
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED off by making the voltage HIGH
  delay(2000);
}