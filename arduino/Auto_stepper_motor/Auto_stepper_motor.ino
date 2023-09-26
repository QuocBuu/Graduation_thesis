// StepperMotor
#include <StepperMotor.h>
  #define pul       A7 // A7
  #define dir_step  A6 // A6
  #define increase  (0.03)
  // Setup_StepperMotor
  StepperMotor stepper(pul, dir_step);
  // Setting_StepperMotor
  float disPerRound = 1.0 ;// only round: 1.0 round, vitme T8-4mm: 4.0 mm, GT2 Pulley 16 Teeth: 16x2 = 32.0 mm
  int microStep = 1; //1: full step, 2: haft step, ...
  float angleStep = 0.05625; //a step angle of 1.8 degrees with 200 steps per revolution
  float stepsPerUnit = (1 / disPerRound) * 360.0 * microStep / angleStep; //steps/round or steps/mm ...
  float target = 100;
  int count_stepper_motor=0;

#define Upper_limit_switch  25
#define Lower_limit_switch  28
void setup() {
  stepper.setStepsPerUnit(stepsPerUnit);
  stepper.setSpeed(1.0);  //set 2 round/s
  stepper.setStartDirection(1);
  Serial.begin(9600);

  pinMode(Upper_limit_switch, INPUT);
  pinMode(Lower_limit_switch, INPUT);
}

float Max_target = 50;
int test_max_target = 0;
float Min_target = -50;
int test_min_target = 0;
int test = 0;
int k = 1;
void loop() {
  if (test == 0) {
    target = target - increase;
  }
  if (digitalRead(Upper_limit_switch) == 0) {
    target = target - increase;
  }

  if (digitalRead(Lower_limit_switch) == 0) {
    test = 1;
    target = target + increase;
    Min_target = target;
  }

  if (k == 1 && test == 1) {
    target = Min_target;
    delay(10000);
    k = 2;
  }
  else if (k == 2) {
    target = Min_target + 5;
  }
  stepper.moveTo(target);
}

