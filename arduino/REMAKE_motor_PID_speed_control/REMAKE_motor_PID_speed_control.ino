char c;
#include<util/atomic.h>
// Pins motor1
#define DC1ENCA 3
#define DC1ENCB 2
#define DC1PWM 4
#define DC1IN2 5

#define PULSEPERROUND 1700

// Pins motor2
#define DC2ENCA 18
#define DC2ENCB 19
#define DC2PWM 6
#define DC2IN2 7


// globals
long prevT_dc1 = 0;
int posPrev_dc1 = 0;
long prevT_dc2 = 0;
int posPrev_dc2 = 0;
volatile int pos_i_dc1= 0;
volatile int pos_i_dc2= 0;
volatile float velocity_i_dc1 = 0;
volatile float velocity_i_dc2 = 0;
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral1 = 0;
float eintegral2 = 0;
float kp = 1.6;
float ki = 3.82;

void setup() {
  Serial.begin(9600);
//MOTOR1
  pinMode(DC1ENCA,INPUT);
  pinMode(DC1ENCB,INPUT);
  pinMode(DC1PWM,OUTPUT);
  pinMode(DC1IN2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(DC1ENCA),readEncoderDC1,RISING);
//MOTOR2
  pinMode(DC2ENCA,INPUT);
  pinMode(DC2ENCB,INPUT);
  pinMode(DC2PWM,OUTPUT);
  pinMode(DC2IN2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(DC2ENCA),readEncoderDC2,RISING);            
}
void loop() {
  /*//ĐK
  if (BLUETOOTH.available())
  {
    c= BLUETOOTH.read();
    Serial.write(c);
  }
  if (Serial.available())
  { c= Serial.read();
  Serial.write(c);
  BLUETOOTH.write(c);
  }
  
 switch (c)
 {
    case '1':// tiến
      a = 10;
      break;
    case '2':// tiến
      a = 12;
      break;
    case '3':// tiến
      a = 14;
      break;
    case '4':// tiến
      a = 16;
      break;
    case '5':// tiến
      a = 18;
      break;
    case '6':// tiến
      a = 20;
      break;
    case '8':// tiến
      a = 23;
      break;
    case '9':// tiến
      a = 25;
      break;  
    case 'F':// tiến
      dir =3;
      vt1 = a;
      vt2 = a;
      break;
    case 'B':// lùi
      dir =2;
      vt1 = a;
      vt2 = a;
      break;
     case 'R': //sang phải
      dir =-1;
      vt1 = a;
      vt2 = a;
      break;
     case 'L':// sang trái
      dir =1;
      vt1 = a;
      vt2 = a;
      break;
    case 'S':// dung
      vt1 = 0;
      vt2 = 0;
      break;
    defailt:
      break;
  }
*/
  // read the position and velocity
  int pos1 = 0;
  int pos2 = 0;
  float vel_at_1 = 0;
  float vel_at_2 = 0;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos1 = pos_i_dc1;
    pos2 = pos_i_dc2;
    vel_at_1 = velocity_i_dc1;
    vel_at_2 = velocity_i_dc2;
  }
//Compute velocity with method 1
  long currT1 = micros();
  float deltaT1 = ((float)(currT1 - prevT_dc1))/1.0e6;
  float velocity1 = (pos1 - posPrev_dc1)/deltaT1;
  posPrev_dc1 = pos1;
  prevT_dc1 = currT1;

//Compute velocity with method 2
  long currT2 = micros();
  float deltaT2 = ((float)(currT2 - prevT_dc2))/1.0e6;
  float velocity2 = (pos2 - posPrev_dc2)/deltaT2;
  posPrev_dc2 = pos2;
  prevT_dc2 = currT2;

//Convert count/s to RPM
  float v1 = velocity1/PULSEPERROUND*60.0;
  float v2 = velocity2/PULSEPERROUND*60.0;

//Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

//set a target
  float vt1= 5*(sin(currT2/1e6)>0);
  float vt2= 20*(sin(currT2/1e6)>0);
// compute the control signal u
  float e1 = vt1 - v1Filt;
  float e2 = vt2 - v2Filt;
  eintegral1 = eintegral1 + e1*deltaT1;
  eintegral2 = eintegral2 + e2*deltaT2;
  float u1 = kp*e1 + ki*eintegral1;
  float u2 = kp*e2 + ki*eintegral2;

  //Set the motor speed and direction
  int dir1 = 1;
  int dir2 = 1;
  if (u1<0){
    dir1 = -1;
  }
  if (u2<0){
    dir2 = -1;
  }
  int pwr1 = (int) fabs(u1);
  if(pwr1 > 255){
    pwr1 = 255;
  } 
  int pwr2 = (int) fabs(u2);
  if(pwr2 > 255){
    pwr2 = 255;
  }

  setMotor(dir1,pwr1,DC1PWM,DC1IN2);
  setMotor(dir2,pwr2,DC2PWM,DC2IN2);
  
// toc do dc  
  Serial.print(vt2);
  Serial.print(" ");
  Serial.print(v2Filt);
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

void readEncoderDC1(){
  // Read encoder B when ENCA rises
  int b1 = digitalRead(DC1ENCB);
  int increment1 = 0;
  if(b1>0){
    // If B is high, increment forward
    increment1 = 1;
  }
  else{
    // Otherwise, increment backward
    increment1 = -1;
  }
  pos_i_dc1 = pos_i_dc1 + increment1;

}
void readEncoderDC2(){
  // Read encoder B when ENCA rises
  int b2 = digitalRead(DC2ENCB);
  int increment2 = 0;
  if(b2>0){
    // If B is high, increment forward
    increment2 = 1;
  }
  else{
    // Otherwise, increment backward
    increment2 = -1;
  }
  pos_i_dc2 = pos_i_dc2 + increment2;

}
