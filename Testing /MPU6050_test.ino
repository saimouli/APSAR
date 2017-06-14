/********************
 * I2C Library
 *********************/
#include "I2Cdev.h"
#include "Wire.h"
/********************
 * MPU6050 Library
 *********************/
#include "MPU6050.h"

/********************
 * Pins
 *********************/
 /*
  * vcc= 5V
  * Gnd= gnd
  * scl= A5
  * sda= A4
  * xda= 
  * xcl=
  * add=Gnd
  * int= 2
  */
MPU6050 accelgyro;

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  Wire.begin();
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  update_MPU6050();
}

void update_MPU6050() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
  
}

