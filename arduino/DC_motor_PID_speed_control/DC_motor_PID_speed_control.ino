#include<util/atomic.h>
#define ENCA 2
#define ENCB A1
#define PWM 13
#define IN2 46
int pwr;
//globals
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;
float eintegral = 0;

void setup() {
  Serial.begin (9600);
   pinMode(ENCA,INPUT);
   pinMode(ENCB,INPUT); 
   pinMode(PWM,OUTPUT);
   pinMode(IN2,OUTPUT);

   attachInterrupt(digitalPinToInterrupt(ENCB),readEncoder,RISING);
   
}

void loop() {
 // int pwr = 50/3.0*micros()/1.0e6;
  /*int pwr = 0;
  int dir =1 ;
  setMotor(dir,pwr,PWM,IN2);*/

  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }
//Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float)(currT - prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

//Convert count/s to RPM
float v1 = velocity1/600.0*60.0;
float v2 = velocity2/600.0*60.0;

//Low-pass filter (25 Hz cutoff)
v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
v1Prev = v1;
v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
v2Prev = v2;

//set a target
float vt= 150*(sin(currT/1e6)>0);

// compute the control signal u
float kp = 1.6;
float ki = 3.82;
float e = vt - v1Filt;
eintegral = eintegral + e*deltaT;
float u = kp*e + ki*eintegral;

//Set the motor speed and direction
int dir = 1;
if (u<0){
  dir = -1;
}
int pwr = (int) fabs(u);
if(pwr > 255){
  pwr = 255;
}
setMotor(dir, pwr, PWM, IN2);
  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
}
void setMotor(int dir, int pwmVal, int pwm, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in2,HIGH);
  }
  
}
void readEncoder(){
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  pos_i = pos_i + increment;

  //compute velocity with method 2
  long currT = micros();
  float deltaT = ((float)(currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
