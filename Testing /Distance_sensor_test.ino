/*
 Elegoo Ping distance sensor:
 VCC to arduino 5v 
 GND to arduino GND
 Echo to Arduino pin 7 
 Trig to Arduino pin 8
 
 Author: Saimouli Katragadda 
 Date: June 11th 2017
 */

#define echoPin 7 // Echo Pin
#define trigPin 8 // Trigger Pin

int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance

void setup() {
 Serial.begin (9600);
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
}

void loop() {
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(trigPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin, LOW); 
 duration = pulseIn(echoPin, HIGH); //Reads a pulse (either HIGH or LOW) on a pin in micro sec.
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance = microsecondstocm(duration);
 
 // output values
Serial.print (distance);
Serial.print ("\n");
 
 //Delay 50ms before next reading.
 delay(50);
}

long microsecondstocm(long duration) {
  long dist;
   //Calculate the distance (in cm) based on the speed of sound.
  dist= duration/58.2;
  return dist;
}
