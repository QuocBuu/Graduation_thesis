#include <StepperMotor.h>
#define pul A7
#define dir A6
#include <SoftwareSerial.h>
SoftwareSerial BLUETOOTH(A15, A14);// pin3 Arduino nối với TX của HC05, pin 2 arduino nối với RX của HC
char c;
StepperMotor stepper(pul, dir);
float disPerRound = 1.0 ;// only round: 1.0 round, vitme T8-4mm: 4.0 mm, GT2 Pulley 16 Teeth: 16x2 = 32.0 mm
int microStep = 1; //1: full step, 2: haft step, ...
float angleStep = 0.05625; //a step angle of 1.8 degrees with 200 steps per revolution
float stepsPerUnit = (1 / disPerRound) * 360.0 * microStep / angleStep; //steps/round or steps/mm ...
float target = 0;
void setup() {
  stepper.setStepsPerUnit(stepsPerUnit);
  stepper.setSpeed(2.0);  //set 2 round/s
  stepper.setStartDirection(HIGH);
  delay(3000);
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
    case 'F':// tiến
      target = target + 0.03;
      break;
    case 'B':// lùi
      target = target - 0.03;
      break;
    case '1':// tiến
      target = 0;
      break;
    case '2':// tiến
      target = 1;
      break;
    case '3':// tiến
      target = 2;
      break;
    case '4':// tiến
      target = 3;
      break;
    case '5':// tiến
      target = 4;
      break;
    case '6':// tiến
      target = 5;
      break;
    case '8':// tiến
      target = 6;
      break;
    case '9':// tiến
      target = 7;
      break;
    defailt:
      break;
  }
  stepper.moveTo(target);
}
