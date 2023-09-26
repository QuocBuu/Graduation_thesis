#include <StepperMotor.h>
#define pul 10
#define dir 11

StepperMotor stepper(pul, dir);

double disPerRound = 1.0 ;// only round: 1.0 round, vitme T8-4mm: 4.0 mm, GT2 Pulley 16 Teeth: 16x2 = 32.0 mm
double microStep = 2; //1: full step, 2: haft step, ...
double angleStep = 1.8; //a step angle of 1.8 degrees with 200 steps per revolution
double stepsPerUnit = (1 / disPerRound) * 360.0 * microStep / angleStep; //steps/round or steps/mm ...
double target = 0;

void setup() {
  stepper.setStepsPerUnit(stepsPerUnit);
  stepper.setSpeed(2.0);  //set 2 round/s
  stepper.setStartDirection(LOW);
  delay(3000);
}

void loop() {
  target = 2;
  stepper.moveTo(target);
  delay(1000);
  target = 0;
  stepper.moveTo(target);
  delay(2000);
}
