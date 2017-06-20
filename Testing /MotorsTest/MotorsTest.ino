// Right encoder
# define Rigth_Encoder_PinA 7
# define Right_Encoder_PinB 4

volatile long Right_Encoder_Ticks= 0;
volatile bool Right_EncoderBSet;

void setup() {
  Serial.begin(115200);
  //pinMode(4,OUTPUT); // LED
  //left motor 
  pinMode(4,OUTPUT); //
  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT); //pwm
  //right right
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(9,OUTPUT);//pwm

  //Encoders
  SetupEncoders();
  
}

/* Encoder code */
void SetupEncoders(){
  //Right 
  pinMode(Rigth_Encoder_PinA,INPUT_PULLUP);
  pinMode(Right_Encoder_PinB,INPUT_PULLUP);

  attachInterrupt(Rigth_Encoder_PinA, do_Right_Encoder,RISING);
  
}
/*End of Encoder functions*/

void goForward(int duration, int pwm)
{
   digitalWrite(4,HIGH);
   delay(duration);//100
   digitalWrite(7,LOW);
   analogWrite(6,pwm);//right motor
  
   digitalWrite(13,HIGH);
   delay(duration);
   digitalWrite(12,LOW);
   analogWrite(9,pwm);// right motor 
}

void fullReverse(int duration, int pwm)
{
   digitalWrite(4,LOW);
   delay(duration);
   digitalWrite(7,HIGH);
   analogWrite(6,pwm);
  
   digitalWrite(13,LOW);
   delay(duration);
   digitalWrite(12,HIGH);
   analogWrite(9,pwm);// right motor
    
}

void fullStop(int duration, int pwm)
{
   digitalWrite(4,LOW);
   delay(duration);
   digitalWrite(7,LOW);
   analogWrite(6,pwm);// 
  
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
  
   digitalWrite(4,HIGH);
   delay(duration);
   digitalWrite(7,LOW);
   analogWrite(6,pwm);
    
}

void turnRight( int duration, int pwm)
{
   digitalWrite(13,HIGH); 
   delay(duration);
   digitalWrite(12,LOW);
   analogWrite(9,pwm);// right motor
  
   digitalWrite(4,LOW);
   delay(duration);
   digitalWrite(7,HIGH);
   analogWrite(6,pwm);
  
}

void loop(){
  Update_Encoders();
  goForward(100,80);
  }

void Update_Encoders(){
  Serial.print ("e");
  Serial.print ("\t");
  Serial.print (Right_Encoder_Ticks); 
  Serial.print ("\n");
}

void do_Right_Encoder(){
  Right_EncoderBSet= digitalRead(Right_Encoder_PinB );
  //read the input pin
  Right_Encoder_Ticks += Right_EncoderBSet ? -1 :+1;
}

