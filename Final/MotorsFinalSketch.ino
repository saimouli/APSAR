/**
  Name:MotorsFinalSketch
  Purpose: Drives Motors and reads encoder values 

  @author SAIMOULI KATRAGADDA
  @version 1.0 7/2/17
*/

// Right encoder
# define Rigth_Encoder_PinA 0
# define Right_Encoder_PinB 1

//left encoder 
# define Left_Encoder_PinA 8
# define Left_Encoder_PinB 9

volatile long Right_Encoder_Ticks= 0;
volatile bool Right_EncoderBSet;

volatile long Left_Encoder_Ticks= 0;
volatile bool Left_EncoderBSet;

/***************
 * PINS
**************/
# define leftPWM 4
# define leftpin1 11
# define leftpin2 10

# define rightPWM 2
# define rightpin1 6
# define rightpin2 7

void setup() {
  Serial.begin(115200);

  //left motor 
  pinMode(leftpin1,OUTPUT); //
  pinMode(leftpin2,OUTPUT);
  pinMode(leftPWM,OUTPUT); //pwm
  //right right
  pinMode(rightpin1,OUTPUT);
  pinMode(rightpin2,OUTPUT);
  pinMode(rightPWM,OUTPUT);//pwm

  //Encoders
  SetupEncoders();
  
}

/* Encoder code */
void SetupEncoders(){
  //Right 
  pinMode(Rigth_Encoder_PinA,INPUT_PULLUP);
  pinMode(Right_Encoder_PinB,INPUT_PULLUP);

  attachInterrupt(1, do_Right_Encoder,RISING);

  //Left
  pinMode(Left_Encoder_PinA,INPUT_PULLUP);
  pinMode(Left_Encoder_PinB,INPUT_PULLUP);

  attachInterrupt(9, do_Left_Encoder,RISING); //catches the pluses
}
/*End of Encoder functions*/

void goForward(int duration, int pwm)
{
   digitalWrite(leftpin1,HIGH);
   delay(duration);//100
   digitalWrite(leftpin2,LOW);
   analogWrite(leftPWM,pwm);//right motor
  
   digitalWrite(rightpin1,HIGH);
   delay(duration);
   digitalWrite(rightpin2,LOW);
   analogWrite(rightPWM,pwm);// right motor 
}

void fullReverse(int duration, int pwm)
{
   digitalWrite(leftPWM,LOW);
   delay(duration);
   digitalWrite(rightpin2,HIGH);
   analogWrite(rightpin1,pwm);
  
   digitalWrite(13,LOW);
   delay(duration);
   digitalWrite(12,HIGH);
   analogWrite(9,pwm);// right motor
    
}

void fullStop(int duration, int pwm)
{
   digitalWrite(leftPWM,LOW);
   delay(duration);
   digitalWrite(rightpin2,LOW);
   analogWrite(rightpin1,pwm);// 
  
   digitalWrite(13,LOW);
   delay(duration);
   digitalWrite(12,LOW);
   analogWrite(9,pwm);// right motor
}

void turnLeft(int duration, int pwm)
{
   digitalWrite(13,LOW); 
   delay(duration);
   digitalWrite(12,HIGH);
   analogWrite(9,pwm);// right motor
  
   digitalWrite(leftPWM,HIGH);
   delay(duration);
   digitalWrite(rightpin2,LOW);
   analogWrite(rightpin1,pwm);
    
}

void turnRight( int duration, int pwm)
{
   digitalWrite(13,HIGH); 
   delay(duration);
   digitalWrite(12,LOW);
   analogWrite(9,pwm);// right motor
  
   digitalWrite(leftPWM,LOW);
   delay(duration);
   digitalWrite(rightpin2,HIGH);
   analogWrite(rightpin1,pwm);
  
}

void loop(){
  Update_Encoders();
  goForward(100,80);
  }

void Update_Encoders(){
  Serial.print ("e");
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

