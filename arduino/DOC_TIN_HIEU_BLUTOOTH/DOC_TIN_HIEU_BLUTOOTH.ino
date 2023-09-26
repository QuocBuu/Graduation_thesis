#include <SoftwareSerial.h>
SoftwareSerial BLUETOOTH(2, 3);// pin3 Arduino nối với TX của HC05, pin 2 arduino nối với RX của HC
char c;
int dir_DC1;
int dir_DC2;
int pwr;
int a;
int dir;
float vt_DC1;
float vt_DC2;
void setup() {
     pinMode(A0, OUTPUT);
     pinMode(A1, OUTPUT);
     pinMode(A2, OUTPUT);
     pinMode(13, OUTPUT);
     pinMode(12, OUTPUT);
     pinMode(11, OUTPUT);
     pinMode(10, OUTPUT);
     pinMode(8, OUTPUT);
     pinMode(9, OUTPUT);
     digitalWrite(8, HIGH);
     digitalWrite(9, HIGH);

     Serial.begin(9600);
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
      a = 20;
      break;
    case '2':// tiến
      a = 30;
      break;
    case '3':// tiến
      a = 40;
      break;
    case '4':// tiến
      a = 50;
      break;
    case '5':// tiến
      a = 60;
      break;
    case '6':// tiến
      a = 70;
      break;
    case '8':// tiến
      a = 80;
      break;
    case '9':// tiến
      a = 90;
      break;  
    case 'F':// tiến
      dir =3;
      vt_DC1 = a;
      vt_DC2 = a;
      break;
    case 'B':// lùi
      dir =2;
      vt_DC1 = a;
      vt_DC2 = a;
      break;
     case 'R': //sang phải
      dir =-1;
      vt_DC1 = a;
      vt_DC2 = a;
      break;
     case 'L':// sang trái
      dir =1;
      vt_DC1 = a;
      vt_DC2 = a;
      break;
    case 'S':// dung
      vt_DC1 = 0;
      vt_DC2 = 0;
      break;
    defailt:
      break;
  }
}
