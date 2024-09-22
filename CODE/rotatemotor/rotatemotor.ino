// Define pins for the ESP8266
int D0=16,D1 =5,D2=4,D3=0,D4=2,D5=14,D6=13,D7=12;
const int dirPin = D1;  // GPIO 5
const int stepPin = D2; // GPIO 4
const int stepsPerRevolution = 200;

void setup()
{
  // Declare pins as Outputs
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);  
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}
int count=0;

void loop()
{
    // while (Serial.available() == 0) {
    // }
    // int d = Serial.parseInt() % 2; // First value for direction (assuming binary 0 or 1)

    // // Clear any residual characters in the input buffer (if needed)
    // while (Serial.available() > 0) {
    //   Serial.read();
    // }

    // // Wait for second input to be available
    // while (Serial.available() == 0) {
    // }
    // int x = Serial.parseInt(); // Second value for steps
  int d=0;
  int x=0;
  if(count%2){
    d=0;
    x=200;
  }else{
    d=1;
    x=100;
  }
    // Print the input values
    Serial.println("Direction and steps:");
    Serial.println(d);  // Print direction (0 or 1)
    Serial.println(x);  // Print steps
    rotate(d, x);
    digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
    delay(1000);                      // Wait for a second
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    delay(2000); 
    count++;  
}

void rotate(bool dir,int steps){
   // Set motor direction clockwise
  digitalWrite(dirPin, (dir==1 ? HIGH : LOW)); // true for cw 

  // Spin motor slowly
  for (int x = 0; x < steps; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000); // 2ms delay
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000); // 2ms delay
  }
  delay(1000); // Wait a second
}