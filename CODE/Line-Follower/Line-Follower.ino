#include <AccelStepper.h>

int D0=16,D1 =5,D2=4,D3=0,D4=2,D5=14,D6=12,D7=13,D8=15;

// const int dirPin = D1;  
// const int stepPin = D2; 

// const int dirPin2 = D5;  
// const int stepPin2 = D0; 
const int irSensorPin = 7;
// const int irSensorPinL = D8;
// const int irSensorPinR = D6;


// #define motorInterfaceType 1

// AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
// AccelStepper myStepper2(motorInterfaceType, stepPin2, dirPin2);

void setup() {
  Serial.begin(9600); 
  pinMode(irSensorPin, INPUT);  // Set the IR sensor pin as input
  // pinMode(irSensorPinL, INPUT);
  // pinMode(irSensorPinR, INPUT);
	// myStepper.setMaxSpeed(1000);
	// myStepper.setAcceleration(500);
	// myStepper.setSpeed(200);
	// myStepper.moveTo(200);

  // myStepper2.setMaxSpeed(1000);
	// myStepper2.setAcceleration(500);
	// myStepper2.setSpeed(200);
	// myStepper2.moveTo(200);
}
// int dist=500;
void loop() {
   int IRS = digitalRead(irSensorPin);  // Read the IR sensor value
  Serial.println(IRS); //This will print the sensor data to serial monitor.
  // int IRL = digitalRead(irSensorPinL);  // Read the IR sensor value
  // Serial.println(IRL);
  // int IRR = digitalRead(irSensorPinR);  // Read the IR sensor value
  // Serial.println(IRR);
  //Giving 1 at Black Color.
  // Line follower logic based on IR sensor reading

//   if(IRS==1){
//   myStepper.move(dist);
//   myStepper2.move(-dist);

//   myStepper.setSpeed(200);
//   myStepper2.setSpeed(-200);
// 	while(myStepper.distanceToGo() != 0) {
//       myStepper.run();
//       myStepper2.run();
//     }
//   }
//   else if(IRL==1){
//   myStepper.move(dist);
//   myStepper2.move(-dist);

//   myStepper.setSpeed(200);
//   myStepper2.setSpeed(0);
// 	while(myStepper.distanceToGo() != 0) {
//       myStepper.run();
//       myStepper2.run();
//     }
//   }

// else if(IRR==1){
//   myStepper.move(dist);
//   myStepper2.move(-dist);

//   myStepper.setSpeed(0);
//   myStepper2.setSpeed(-200);
// 	while(myStepper.distanceToGo() != 0) {
//       myStepper.run();
//       myStepper2.run();
//     }
  // }
// else{
//   myStepper.move(dist);
//   myStepper2.move(-dist);

//   myStepper.setSpeed(0);
//   myStepper2.setSpeed(0);
// 	while(myStepper.distanceToGo() != 0) {
//       myStepper.run();
//       myStepper2.run();
//     }
// }
}