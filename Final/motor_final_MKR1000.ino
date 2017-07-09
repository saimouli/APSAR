/*
#  APSAR Project
#  
#  Copyright 2017 Saimouli Katragadda <skatraga@umd.edu>
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
# for more info. see testing folder from APSAR github
# Some of the portion is adapted from Lentin Joseph's Chefbot and I2c example code
*/

// ================================================================
// ===               LIBRARIES                                  ===
// ================================================================

/*The MPU6050 IMU contains a DMP (Digital Motion Processor) which fuses 
the accelerometer and gyroscope data together to minimize the effects 
of errors inherent in each sensor.*/
//Processing incoming serial data 
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

// ================================================================
// ===                    INITIALIZE                     ===
// ================================================================
//Messenger object
Messenger Messenger_Handler = Messenger();

// ================================================================
// ===                    MOTORS SETUP PINS                     ===
// ================================================================
// Right encoder
# define Rigth_Encoder_PinA 0
# define Right_Encoder_PinB 1

//left encoder 
# define Left_Encoder_PinA 8
# define Left_Encoder_PinB 9

//encoder interrput pins
# define Encoder_inter_pin1 12 
# define Encoder_inter_pin2 13

volatile long Right_Encoder_Ticks= 0;
volatile bool Right_EncoderBSet;

volatile long Left_Encoder_Ticks= 0;
volatile bool Left_EncoderBSet;

/***************
 * PINS
****************/
# define leftPWM 4
# define leftpin1 11
# define leftpin2 10

# define rightPWM 2
# define rightpin1 6
# define rightpin2 7

# define INTERRUPT_PIN 14 // for IMU pin

// ================================================================
// ===                    ULTRASONIC PINS                       ===
// ================================================================
#define echoPin 3 // Echo Pin
#define trigPin 5 // Trigger Pin

int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables

unsigned long LastUpdateMicrosecs = 0;    
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

///////////////////////////////////////////////////////////////////////////////////////
//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;

// ================================================================
// ===                     VOID SETUP                           ===
// ================================================================
void setup() {
  Serial.begin(115200);
  
  //Encoders
  SetupEncoders();

  //setupMotors
  setupMotors();

  //Ultrasonic senors
  DistanceSensor();

  //Set up Messenger 
  Messenger_Handler.attach(OnMssageCompleted);
}

//////////////////////////////////////////////////////////////
void SetupEncoders(){
  //Right 
  pinMode(Rigth_Encoder_PinA,INPUT_PULLUP);
  pinMode(Right_Encoder_PinB,INPUT_PULLUP);

  attachInterrupt(Encoder_inter_pin1, do_Right_Encoder,RISING); //catches the pluses

  //Left
  pinMode(Left_Encoder_PinA,INPUT_PULLUP);
  pinMode(Left_Encoder_PinB,INPUT_PULLUP);

  attachInterrupt(Encoder_inter_pin2, do_Left_Encoder,RISING); //catches the pluses
}

//////////////////////////////////////////////////////////////
void setupMotors() {
  //left motor 
  pinMode(leftpin1,OUTPUT); //
  pinMode(leftpin2,OUTPUT);
  pinMode(leftPWM,OUTPUT); //pwm
  //right right
  pinMode(rightpin1,OUTPUT);
  pinMode(rightpin2,OUTPUT);
  pinMode(rightPWM,OUTPUT);//pwm
}

//////////////////////////////////////////////////////////////
void DistanceSensor(){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


/////////////////////////////////////////////////////////////////////////////////
// ================================================================
// ===                     MAIN LOOP                           ===
// ================================================================

void loop(){
  //Read from Serial port
  Read_From_Serial();
    
  //Send time information through serial port
  Update_Time();
  
  // update encoders
  Update_Encoders();

  //update motors
  Update_Motors();

  //update ultrasonic sensor
  Update_DistanceSensor();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial(){
   while(Serial.available() > 0){
       int data = Serial.read();
       Messenger_Handler.process(data);
    } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted(){
  char reset[] = "r";
  char set_speed[] = "s";
  
  if(Messenger_Handler.checkString(reset)){
    
     Serial.println("Reset Done"); 
     //TO DO: reserach about uno reset pin
    
  }
  if(Messenger_Handler.checkString(set_speed)){
     //This will set the speed
     Set_Speed();
     return; 
  }
}
////////////////////////////////////////////////////////////////////////////
void Update_Encoders(){
  Serial.print ("e"); // e for encoders
  Serial.print ("\t");
  Serial.print (Right_Encoder_Ticks); 
  Serial.print ("\t");
  Serial.print (Left_Encoder_Ticks); 
  Serial.print ("\n");
}

void do_Right_Encoder(){
  Right_EncoderBSet= digitalRead(Right_Encoder_PinB );
  //read the input pin
  Right_Encoder_Ticks += Right_EncoderBSet ? -1 :+1;
}

void do_Left_Encoder(){
  Left_EncoderBSet= digitalRead(Left_Encoder_PinB );
  //read the input pin
  Left_Encoder_Ticks += Left_EncoderBSet ? -1 :+1;
}
///////////////////////////////////////////////////////////////////////////
//Set speed
void Set_Speed(){    
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
}

////////////////////////////////////////////////////////////////////////////
//Will update both motors
void Update_Motors(){
  
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);

  Serial.print("s"); //s for speed
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);  
  Serial.print("\n");
}

// <write motor functions>
void moveRightMotor(float speedvalue1){ // for the logic below, see motor final sketch
  if(speedvalue1>0){
    digitalWrite(rightpin1,HIGH);
    digitalWrite(rightpin2,LOW);
    analogWrite(rightPWM,speedvalue1);// right motor 
  }
  else if(speedValue<0){ // reverse
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,HIGH);
    analogWrite(rightPWM,abs(speedvalue1));// right motor
  } 
  else if (speedValue==0){ //stop
    digitalWrite(rightpin1,LOW);
    digitalWrite(rightpin2,LOW);
    analogWrite(rightPWM,speedvalue1);// right motor
  } 
}

void moveLeftMotor(float speedvalue2){
  if(speedvalue2>0){
   digitalWrite(leftpin1,HIGH);
   digitalWrite(leftpin2,LOW);
   analogWrite(leftPWM,speedvalue2);//left motor
  }
  else if(speedValue<0){ // reverse
   digitalWrite(leftpin1,LOW);
   digitalWrite(leftpin2,HIGH);
   analogWrite(leftPWM,abs(speedvalue2));
  } 
  else if (speedValue==0){ //stop
   digitalWrite(leftpin1,LOW);
   digitalWrite(leftpin2,LOW);
   analogWrite(leftPWM,speedvalue2);
  } 
  
}
////////////////////////////////////////////////////////////////////////////
void Update_DistanceSensor(){
 digitalWrite(trigPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin, LOW); 
 duration = pulseIn(echoPin, HIGH); //Reads a pulse (either HIGH or LOW) on a pin in micro sec.
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance = microsecondstocm(duration);
 
 // output values
 //Sending through serial port
  Serial.print("d"); // d for distance sensor
  Serial.print("\t");
  Serial.print (distance);
  Serial.print("\n");
  
 //Delay 50ms before next reading.
 delay(50);
}

long microsecondstocm(long duration) {
  long dist;
   //Calculate the distance (in cm) based on the speed of sound.
  dist= duration/58.2;
  return dist;
}


