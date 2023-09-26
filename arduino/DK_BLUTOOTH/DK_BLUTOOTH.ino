#include <SoftwareSerial.h>
SoftwareSerial BLUETOOTH(A15, A14);// pin3 Arduino nối với TX của HC05, pin 2 arduino nối với RX của HC
char c;
int pwr;
int a=80;
int dir;
int PWM_DC1 =13;
int PWM_DC2 =45;
int IN2_DC1 =46;
int IN2_DC2 =44;
void setup() {
    Serial.begin(9600);
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
      break;
    case '2':// tiến
      a = 120;
      break;
    case '3':// tiến
      a = 140;
      break;
    case '4':// tiến
      a = 160;
      break;
    case '5':// tiến
      a = 180;
      break;
    case '6':// tiến
      a = 200;
      break;
    case '8':// tiến
      a = 220;
      break;
    case '9':// tiến
      a = 255;
      break;  
    case 'F':// tiến
      dir =-1;
      pwr = a;
      break;
    case 'B':// lùi
      dir =1;
      pwr = a;
      break;
     case 'R': //sang phải
      dir =3;
      pwr = a;
      break;
     case 'L':// sang trái
      dir =2;
      pwr = a;
      break;
    case 'S':// dung
      pwr = 0;
      break;
    defailt:
      pwr = 0;
      break;
  }
  setMotor(dir,pwr,PWM_DC1,IN2_DC1,IN2_DC2);
  setMotor(dir,pwr,PWM_DC2,IN2_DC1,IN2_DC2);
  
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
