#include <WiFi.h>
#include <WebServer.h>

// Replace with your own credentials
const char *ssid = "ESP32-Car-Control";
const char *password = "12345678";  // Password for the hotspot

// Initialize web server on port 80
WebServer server(80);

// Movement control variables
String carDirection = "STOP";

// Define motor control pins (replace with actual pins)
const int motorPin1 = 5;  // Motor A
const int motorPin2 = 18; // Motor A


// Function to handle the control
void handleControl() {
  if (server.hasArg("direction")) {
    carDirection = server.arg("direction");

    // Control the motors based on direction
    controlCar();

    // Send response (can be ignored by the client)
    server.send(200, "text/plain", "OK");
  }
}

// Function to control car movement
void controlCar() {
  if (carDirection == "FORWARD") {
    // Set motors to move forward
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, 1);

  } else if (carDirection == "BACKWARD") {
    // Set motors to move backward
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, 0);

  } else if (carDirection == "LEFT") {
    // Set motors to rotate left
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);

  } else if (carDirection == "RIGHT") {
    // Set motors to rotate right
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);

  } else if (carDirection == "STOP") {
    // Stop the motors
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    Serial.println("Stopping");
  }
}

// Function to handle root path "/"
void handleRoot() {
  String html = "<html><body><h1>ESP32 Car Control</h1>";
  html += "<button onclick=\"sendData('FORWARD')\">Forward</button><br><br>";
  html += "<button onclick=\"sendData('BACKWARD')\">Backward</button><br><br>";
  html += "<button onclick=\"sendData('LEFT')\">Rotate Left</button><br><br>";
  html += "<button onclick=\"sendData('RIGHT')\">Rotate Right</button><br><br>";
  html += "<button onclick=\"sendData('STOP')\">Stop</button><br><br>";
  html += "<script>";
  html += "function sendData(direction) {";
  html += "var xhttp = new XMLHttpRequest();";
  html += "xhttp.open('POST', '/control?direction=' + direction, true);";
  html += "xhttp.send();";
  html += "}";
  html += "</script></body></html>";

  server.send(200, "text/html", html);
}

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  
  // Set motor pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);


  // Set ESP32 as Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");

  // Print IP address
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Define routes
  server.on("/", handleRoot);
  server.on("/control", handleControl);

  // Start server
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // Handle incoming client requests
  server.handleClient();
}