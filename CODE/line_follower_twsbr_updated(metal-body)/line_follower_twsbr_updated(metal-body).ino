#include <Wire.h>                                          

int gyro_address = 0x68;                                   
int acc_calibration_value = -660;                        

//Various settings
float pid_p_gain = 13;    //13                                  
float pid_i_gain = 0.55; //1                                     
float pid_d_gain = 5;  //3.6                                     
float turning_speed = 10;                                    
float turning_speed_slow = 5;                               
float max_target_speed = 4;                                

byte start, received_byte=0, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long ir_timer_start;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro,angle_gyro_yaw=0, angle_acc=0, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

// const int irPin1=10;
// const int irPin2=11;
// const int irPin3=12;

// const int irPin4=2;
// const int irPin5=3;

// const int irPin6=4; //right 90
// const int irPin7=9; //left 90

const int irPin1=2;//left 90
const int irPin2=3;
const int irPin3=4;

const int irPin4=9;
const int irPin5=10;

const int irPin6=11; 
const int irPin7=12; //right 90

void setup(){
  Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode


  // 1sec timer 1 
  // Configure Timer1 for CTC mode
  TCCR1A = 0;                 // Clear Timer/Counter Control Register A
  TCCR1B = 0;                 // Clear Timer/Counter Control Register B
  TCCR1B |= (1 << WGM12);     // Set WGM12 bit for CTC mode
  
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set prescaler to 1024 
  // Set compare match value for 1 second
  OCR1A = 15624;              // 1-second interval with 16 MHz clock and 1024 prescaler
  // OCR1A = 7812;  //0.5 sec timer
  // OCR1A = 6250;   //0.4sec timer
  TIMSK1 |= (1 << OCIE1A);    // Enable Timer1 Compare Match A interrupt
  // sei();    // Enable global interrupts

  
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro 

  pinMode(13, OUTPUT);                                                      //Configure digital poort 6 as output
  
  pinMode(5, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(6, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(7, OUTPUT);                                                       //Configure digital poort 4 as output
  pinMode(8, OUTPUT); 

  
  pinMode(irPin1, INPUT);                                                   
  pinMode(irPin2, INPUT);                                                   
  pinMode(irPin3, INPUT);       
  pinMode(irPin4, INPUT);       
  pinMode(irPin5, INPUT); 
  pinMode(irPin6, INPUT); 
  pinMode(irPin7, INPUT);        
  
  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

}

int counter=0;
int time_counter=0;

int timetorun=750;
int turning=0;
long turningOffset=0;
long turningFinal=0;

int flagIR=0;
bool started=false;
bool noIRdet=false;
int cntNoIRdetTime=0;

int timeFirstDetected=0;
int leftdetected = 0, rightdetected = 0;
int leftdetectedAgain = 0, rightdetectedAgain = 0;

void loop(){
  counter++; // 4ms
  time_counter++;

  if(time_counter>=timetorun){
    time_counter=0;
  }
  // if(Serial.available()){                                                   //If there is serial data available
  //   received_byte = Serial.read();                                          //Load the received serial data in the received_byte variable
  //   receive_counter = 0;                                                    //Reset the receive_counter variable
  // }

    // int IRL = digitalRead(irPin1) | digitalRead(irPin4);
    // int IRS = digitalRead(irPin2);
    // int IRR = digitalRead(irPin3) |digitalRead(irPin5);

    
    // int IRLeft = digitalRead(irPin7);
    // int IRRight = digitalRead(irPin6);

    int IRL = digitalRead(irPin5) | digitalRead(irPin6);
    int IRS = digitalRead(irPin4);
    int IRR = digitalRead(irPin2) |digitalRead(irPin3);

    
    int IRLeft = digitalRead(irPin7);
    int IRRight = digitalRead(irPin1);

    // IRL=0;
    // IRS=0;
    // IRR=0;
    IRLeft=0;
    IRRight=0;

    // Serial.println(IRL);
    // Serial.println(IRS);
    // Serial.println(IRR);
    // Serial.println(IRLeft);
    // Serial.println(IRRight);
    // Serial.println(angle_gyro_yaw);
    
    // Serial.println(received_byte);
    // Serial.print("Left IR ");
    // Serial.println(IRLeft);
    // Serial.print("leftdetected ");
    // Serial.println(leftdetected);
    // Serial.print("leftdetectedAgain ");
    // Serial.println(leftdetectedAgain);
   
   
    if(leftdetected && IRLeft && leftdetectedAgain==0 && timeFirstDetected>1){
      leftdetectedAgain=1;
      leftdetected=0;
      turning=1;
      turningOffset=angle_gyro_yaw;
      turningFinal=turningOffset-90;
      // Serial.print("Left 90");
      
    }else if(rightdetected && IRRight && rightdetectedAgain==0 && timeFirstDetected>1){
      rightdetectedAgain=1;
      rightdetected=0;
      turning=1;
      turningOffset=angle_gyro_yaw;
      turningFinal=turningOffset+90;
      // Serial.print("Right 90");
    }

      //  Serial.print("turning ");
      //  Serial.println(turning);

      //  Serial.print("IRLeft ");
      //  Serial.println(IRLeft);

    if(turning==0 && IRLeft){
      leftdetected=1;
      timeFirstDetected=1;
    }else if(turning==0 && IRRight){
      rightdetected=1;
      timeFirstDetected=1;
    }

    if(!IRL && !IRS && !IRR && !IRLeft && !IRRight && turning==0 && started){
      if(!noIRdet)
      {
        noIRdet=true;
        cntNoIRdetTime=0;
      }
    }else{
      noIRdet=false;
      cntNoIRdetTime=0;
    }

 
  if(counter>=30){
    if(turning==0)
    {
      // Serial.println("turn=0");

      // if(IRLeft){
      //   turning=1;
      //   turningOffset=angle_gyro_yaw; 
      //   turningFinal=turningOffset-90;
      // }else if(IRRight){
      //   turning=1;
      //   turningOffset=angle_gyro_yaw;
      //   turningFinal=turningOffset+90;
      // }
      if(noIRdet && cntNoIRdetTime>=4){
      // if((cntNoIRdetTime%4)>=2){
        if(received_byte==0){
          received_byte = 0x08;
          receive_counter=5;
          counter=0;
        }
      // }
      }
      else if(leftdetected || rightdetected){
        if(received_byte==0){
          received_byte=0x08;
          receive_counter=5;
          counter=0;
        }
      }
      else if(IRS && received_byte==0){
        started=true;
        received_byte = 0x04;
        receive_counter=5;
        counter=0;
      }
      else if(IRL){
        received_byte = 0x01;
        receive_counter=0;
        counter=0;
        // turning=1;
        // turningOffset=angle_gyro_yaw; 
        // turningFinal=turningOffset-5;
      }else if(IRR){
        received_byte = 0x02;
        receive_counter=0;
        counter=0;
        // turning=1;
        // turningOffset=angle_gyro_yaw; 
        // turningFinal=turningOffset+5;
      }
    }
    else if(abs(turningOffset-turningFinal)>=5){
      turningOffset=angle_gyro_yaw;
      // Serial.println("hii ");
      // Serial.print(turningOffset);
      // Serial.print(" ");
      // Serial.println(turningFinal);
      if(turningOffset>turningFinal){
        received_byte = 0x01;
        receive_counter=0;
        // counter=0;
        // Serial.println("Left");
      }else{
        received_byte = 0x02;
        receive_counter=0;
        // counter=0;
        // Serial.println("Right");
      }
    }else{
      turning=0;
      leftdetectedAgain=0;
      rightdetectedAgain=0;
      timeFirstDetected=0;
      // Serial.println("Dhokebaaz");
      // Serial.print(turningOffset);
      // Serial.print(" ");
      // Serial.println(turningFinal);
    }
    // counter=0;
  }

//  Serial.print("Angle Gyro yaw");
//  Serial.println(angle_gyro_yaw);

  if(receive_counter <= 25)receive_counter ++;                              
  //The received byte will be valid for 25 program loops (100 milliseconds)
  else received_byte = 0x00;                                                //After 100 milliseconds the received byte is deleted
  
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
  // if(start == 0){
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    angle_gyro_yaw = angle_acc;
    start = 1;                                                              //Set the start variable to start the PID controller
  }
  
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                           //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                         //Combine the two bytes to make one integer
  
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  
  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  angle_gyro_yaw+=gyro_yaw_data_raw * 0.000031;  
  //Uncomment the following line to make the compensation active
  angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle
  // angle_gyro_yaw = angle_gyro_yaw*0.9996 + angle_acc * 0.0004;

  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  // if(angle_gyro > 60 || angle_gyro < -60 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
  //   pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
  //   pid_i_mem = 0;                                                          //Reset the I-controller memory
  //   start = 0;                                                              //Set the start variable to 0
  //   self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  //   Serial.println("Stopping..........");
  // }

  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if(received_byte & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if(received_byte & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }

  if(received_byte & B00010000){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed_slow;                                       //Increase the left motor speed
    pid_output_right -= turning_speed_slow;                                      //Decrease the right motor speed
  }
  if(received_byte & B00100000){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed_slow;                                       //Decrease the left motor speed
    pid_output_right += turning_speed_slow;                                      //Increase the right motor speed
  }

  if(received_byte & B00000100){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.04;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if(received_byte & B00001000){                                            //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.04;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(received_byte & B00001100)){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0010;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0010;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;
  while(loop_timer > micros());
  loop_timer += 4000;
}

int leftmotorPulse=0,rightmotorPulse=0;
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
      // PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
      PORTB &= 0b11111110;        //Set output 8 low                                          
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    // else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
    else PORTB |= 0b00000001;  //Set output 8 high
  }
  // else if(throttle_counter_left_motor == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
  // else if(throttle_counter_left_motor == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 
  else if(throttle_counter_left_motor == 1){
    PORTD |= 0b01000000;             //Set output 6 high to create a pulse for the stepper controller
    leftmotorPulse++;
  }
  else if(throttle_counter_left_motor == 2)PORTD &= 0b10111111;             //Set output 6 low because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      // PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      PORTD |= 0b10000000; // //Set output 7
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    // else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
     else PORTD &= 0b01111111;      //Set output 7
  }
  // else if(throttle_counter_right_motor == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
  // else if(throttle_counter_right_motor == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
  else if(throttle_counter_right_motor == 1){
    PORTD |= 0b00100000;            //Set output 5 high to create a pulse for the stepper controller
    rightmotorPulse++;
  }
  else if(throttle_counter_right_motor == 2)PORTD &= 0b11011111;            //Set output 5 low because the pulse only has to last for 20us
}

ISR(TIMER1_COMPA_vect) {
//  Serial.print(leftmotorPulse);  // Print the left motor pulse
//  Serial.print(" ");
//   Serial.print(rightmotorPulse); // Print the right motor pulse
//   Serial.print(" ");
//   Serial.println(angle_gyro);    // Print the angle_gyro with a newline    

  leftmotorPulse=0;
  rightmotorPulse=0;
  flagIR = (flagIR+1)%2;
  if(noIRdet){
    cntNoIRdetTime++;
  }
  if(timeFirstDetected!=0) timeFirstDetected++;

}