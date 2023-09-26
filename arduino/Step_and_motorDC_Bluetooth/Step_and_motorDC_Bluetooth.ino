#include <StepperMotor.h>
#define pul A7
#define dir_step A6
#include <SoftwareSerial.h>
SoftwareSerial BLUETOOTH(A15, A14);// pin3 Arduino nối với TX của HC05, pin 2 arduino nối với RX của HC
char c;
StepperMotor stepper(pul, dir_step);
float disPerRound = 1.0 ;// only round: 1.0 round, vitme T8-4mm: 4.0 mm, GT2 Pulley 16 Teeth: 16x2 = 32.0 mm
int microStep = 1; //1: full step, 2: haft step, ...
float angleStep = 0.05625; //a step angle of 1.8 degrees with 200 steps per revolution
float stepsPerUnit = (1 / disPerRound) * 360.0 * microStep / angleStep; //steps/round or steps/mm ...
float target = 0;
int pwr;
int a=80;
int b=0;
int d=0;
int dir;
int PWM_DC1 =13;
int PWM_DC2 =45;
int IN2_DC1 =46;
int IN2_DC2 =44;
void setup() {
    Serial.begin(9600);
  stepper.setStepsPerUnit(stepsPerUnit);
  stepper.setSpeed(2.0);  //set 2 round/s
  stepper.setStartDirection(HIGH);
  delay(3000);
  pinMode(PWM_DC1,OUTPUT);
  pinMode(PWM_DC2,OUTPUT);
  pinMode(IN2_DC1,OUTPUT);
  pinMode(IN2_DC2,OUTPUT);
     BLUETOOTH.begin(9600);
        
}

void loop() {
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
      a = 100;
      target = 0;
      break;
    case '2':// tiến
      a = 120;
      target = 1;
      break;
    case '3':// tiến
      a = 140;
      target = 2;
      break;
    case '4':// tiến
      a = 160;
      target = 3;
      break;
    case '5':// tiến
      a = 180;
      target = 4;
      break;
    case '6':// tiến
      a = 200;
      target = 5;
      break;
    case '8':// tiến
      a = 220;
      target = 6;
      break;
    case '9':// tiến
      a = 255;
      target = 7;
      break;  
    case 'F':// tiến
      dir =-1;
      pwr = a;
      target = target + 0.03;
      break;
    case 'B':// lùi
      dir =1;
      pwr = a;
      target = target - 0.03;
      break;
     case 'R': //sang phải
      dir =3;
      pwr = a;
      break;
     case 'L':// sang trái
      dir =2;
      pwr = a;
      break;
    case 'W':// dung
      target = d;
      b=1;
      break;
    case 'w':// dung
      target = d;
      b=0;
      break;
    case 'S':// dung
      pwr = 0;
      break;
    defailt:
      pwr = 0;
      break;
  }
  if(b == 0){
    setMotor(dir,pwr,PWM_DC1,IN2_DC1,IN2_DC2);
    setMotor(dir,pwr,PWM_DC2,IN2_DC1,IN2_DC2);
  }
  if(b == 1){
    pwr = 0;
    d = target;
    stepper.moveTo(target);
  }
  Serial.print(target);
}
void setMotor(int dir, int pwmVal, int PWM , int IN2_DC1, int IN2_DC2){
  analogWrite(PWM,pwmVal);
  if(dir == 1){
    digitalWrite(IN2_DC1,LOW);
    digitalWrite(IN2_DC2,LOW);
  }
  if(dir == -1){
    digitalWrite(IN2_DC1,HIGH);
    digitalWrite(IN2_DC2,HIGH);
  }
  if(dir == 2){
    digitalWrite(IN2_DC1,LOW);
    digitalWrite(IN2_DC2,HIGH);
  }
  if(dir == 3){
    digitalWrite(IN2_DC1,HIGH);
    digitalWrite(IN2_DC2,LOW);
  }
}
