#include<util/atomic.h>
#include <TimerOne.h>
#define ENCA 2
#define ENCB A1
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;
float eintegral = 0;
double T, xung;
double tocdo, tocdodat;
double E, E1, E2;
double a, b, g, kp, ki, kd;
double Output, LastOutput;
void setup() {
  Serial.begin (9600);
   pinMode(ENCA,INPUT);
   pinMode(ENCB,INPUT); 
   attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
   Timer1.initialize(10000);
   Timer1.attachInterrupt(pid);
}

void loop() {
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
  pid(pos_i);
  //Serial.println(tocdo);
  Serial.println(pos);
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

void pid(int pos_i)
{
  tocdo = (pos_i/320)*(1/T)*60;
}
