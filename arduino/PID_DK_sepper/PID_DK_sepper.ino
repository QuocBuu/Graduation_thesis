#include <SoftwareSerial.h>
#include <StepperMotor.h>
#include <util/atomic.h>
// Stepper 
#define pul       A7
#define dir_step  A6
// Bluetooth 
#define TX        A15
#define RX        A14
// Declare DC1
#define ENCA_DC1 2
#define ENCB_DC1 A1
#define PWM_DC1 13
#define IN2_DC1 46
// Declare DC2
#define ENCA_DC2 3
#define ENCB_DC2 49
#define PWM_DC2 45
#define IN2_DC2 44
// 
StepperMotor stepper(pul, dir_step);
SoftwareSerial BLUETOOTH(TX, RX);
// Setup Stepper
float disPerRound = 1.0 ;// only round: 1.0 round, vitme T8-4mm: 4.0 mm, GT2 Pulley 16 Teeth: 16x2 = 32.0 mm
int microStep = 1; //1: full step, 2: haft step, ...
float angleStep = 0.05625; //a step angle of 1.8 degrees with 200 steps per revolution
float stepsPerUnit = (1 / disPerRound) * 360.0 * microStep / angleStep; //steps/round or steps/mm ...

float target = 0;
char bluetooth_input;  //for receiving bluetooth
int speed_pwm = 50;
int mode = 0;
int count_stepper = 0;
// DC1
int pwr_DC1;
long prevT_DC1 = 0;
int posPrev_DC1 = 0;
volatile int pos_i_DC1 = 0;
volatile float velocity_i_DC1 = 0;
volatile long prevT_i_DC1 = 0;
float eintegral_DC1 = 0;
// DC2
int pwr_DC2;
long prevT_DC2 = 0;
int posPrev_DC2 = 0;
volatile int pos_i_DC2 = 0;
volatile float velocity_i_DC2 = 0;
volatile long prevT_i_DC2 = 0;
float eintegral_DC2 = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

void setup() {
  Serial.begin(9600);
  BLUETOOTH.begin(9600);
  // DC1
  pinMode(ENCA_DC1, INPUT);
  pinMode(ENCB_DC1, INPUT);
  pinMode(PWM_DC1, OUTPUT);
  pinMode(IN2_DC1, OUTPUT);
  // DC2
  pinMode(ENCA_DC2, INPUT);
  pinMode(ENCB_DC2, INPUT);
  pinMode(PWM_DC2, OUTPUT);
  pinMode(IN2_DC2, OUTPUT);
  if (mode == 1) {
    // Stepper
    stepper.setStepsPerUnit(stepsPerUnit);
    stepper.setSpeed(2.0);  //set 2 round/s
    stepper.setStartDirection(HIGH); 
    delay(3000);
  }
  // Encoder A, B  
  attachInterrupt(digitalPinToInterrupt(ENCA_DC1), readEncoder_DC1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_DC2), readEncoder_DC2, RISING);
}

void loop() {
  float vt_DC1;
  float vt_DC2;
  //Dk Bluetooth
  if (BLUETOOTH.available()) {
    bluetooth_input = BLUETOOTH.read();
    Serial.write(bluetooth_input);
  }
  if (Serial.available()) {
    bluetooth_input = Serial.read();
    Serial.write(bluetooth_input);
    BLUETOOTH.write(bluetooth_input);
  }
  switch (bluetooth_input) {
    case '1':  // tiến
      if (mode == 0) {
        speed_pwm = 50;
      }
      else {
        target = 0;
      }
      break;

    case '2':  // tiến
      if (mode == 0) {
        speed_pwm = 80;
      }
      else {
        target = 3;
      }
      break;

    case '3':  // tiến
      if (mode == 0) {
        speed_pwm = 110;
      }
      else {
        target = 6;
      }
      break;

    case '4':  // tiến
      if (mode == 0) {
        speed_pwm = 140;
      }
      else {
        target = 9;
      }      
      break;

    case '5':  // tiến
      if (mode == 0) {
        speed_pwm = 170;
      }
      else {
        target = 12;
      }
      break;

    case '6':  // tiến
      if (mode == 0) {
        speed_pwm = 200;
      }
      else {
        target = 15;
      }
      break;

    case '8':  // tiến
      if (mode == 0) {
        speed_pwm = 230;
      }
      else {
        target = 18;
      }
      break;

    case '9':  // tiến
      if (mode == 0) {
        speed_pwm = 255;
      }
      else {
        target = 21;
      }
      break;

    case 'W':// dung
      target = count_stepper;
      mode=1;
      break;

    case 'w':// dung
      target = count_stepper;
      mode=0;
      break;

    case 'F':  // tiến
      vt_DC1 = -speed_pwm;
      vt_DC2 = speed_pwm;
      break;

    case 'B':  // lùi
      vt_DC1 = speed_pwm;
      vt_DC2 = -speed_pwm;
      break;

    case 'R':  //sang phải
      vt_DC1 = -speed_pwm;
      vt_DC2 = -speed_pwm;
      break;

    case 'L':  // sang trái
      vt_DC1 = speed_pwm;
      vt_DC2 = speed_pwm;
      break;

    case 'S':  // dung
      vt_DC1 = 0;
      vt_DC2 = 0;
      break;

    defailt:
      vt_DC1 = 0;
      vt_DC2 = 0;
      break;
  }
  // PID
  int pos_DC1 = 0;
  float velocity1_DC1 = 0;
  int pos_DC2 = 0;
  float velocity2_DC2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_DC1 = pos_i_DC1;
    velocity1_DC1 = velocity_i_DC1;
    pos_DC2 = pos_i_DC2;
    velocity2_DC2 = velocity_i_DC2;
  }
  //Compute velocity with method 1
  long currT_DC1 = micros();
  float deltaT_DC1 = ((float)(currT_DC1 - prevT_DC1)) / 1.0e6;
  velocity1_DC1 = (pos_DC1 - posPrev_DC1) / deltaT_DC1;
  posPrev_DC1 = pos_DC1;
  prevT_DC1 = currT_DC1;

  //Compute velocity with method 1
  long currT_DC2 = micros();
  float deltaT_DC2 = ((float)(currT_DC2 - prevT_DC2)) / 1.0e6;
  velocity2_DC2 = (pos_DC2 - posPrev_DC2) / deltaT_DC2;
  posPrev_DC2 = pos_DC2;
  prevT_DC2 = currT_DC2;

  //Convert count/s to RPM
  float v1 = velocity1_DC1 / 600.0 * 60.0;
  float v2 = velocity2_DC2 / 600.0 * 60.0;

  //Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;

  //set speed_pwm target
  //float vt_DC1 = 100*sin(currT_DC1 /1e6);
  //float vt_DC2 = 100*sin(currT_DC1 /1e6);

  // compute the control signal u
  float kp_DC1 = 2.2;
  float ki_DC1 = 4.2;
  // compute the control signal u
  float kp_DC2 = 1.9;
  float ki_DC2 = 3.82;

  float e_DC1 = vt_DC1 - v1Filt;
  eintegral_DC1 = eintegral_DC1 + e_DC1 * deltaT_DC1;
  float u_DC1 = kp_DC1 * e_DC1 + ki_DC1 * eintegral_DC1;

  float e_DC2 = vt_DC2 - v2Filt;
  eintegral_DC2 = eintegral_DC2 + e_DC2 * deltaT_DC2;
  float u_DC2 = kp_DC2 * e_DC2 + ki_DC2 * eintegral_DC2;

  //Set the motor speed and direction
  int dir_DC1 = -1;
  if (u_DC1 < 0) {
    dir_DC1 = 1;
  }
  int pwr_DC1 = (int)fabs(u_DC1);
  if (pwr_DC1 > 255) {
    pwr_DC1 = 255;
  }
  //Set the motor speed and direction
  int dir_DC2 = 1;
  if (u_DC2 < 0) {
    dir_DC2 = -1;
  }
  int pwr_DC2 = (int)fabs(u_DC2);
  if (pwr_DC2 > 255) {
    pwr_DC2 = 255;
  }

  // run car
  if(mode == 0){ 
    setMotor(dir_DC1, pwr_DC1, PWM_DC1, IN2_DC1);
    setMotor(dir_DC2, pwr_DC2, PWM_DC2, IN2_DC2);
  }
  if(mode == 1){
    vt_DC1 = 0;
    vt_DC2 = 0;
    count_stepper = target;
    stepper.moveTo(target);
  }
}
void setMotor(int dir, int pwmVal, int pwm, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in2, HIGH);
  }
}
void readEncoder_DC1() {
  int b_DC1 = digitalRead(ENCB_DC1);
  int increment_DC1 = 0;
  if (b_DC1 > 0) {
    increment_DC1 = 1;
  } else {
    increment_DC1 = -1;
  }
  pos_i_DC1 = pos_i_DC1 + increment_DC1;

  //compute velocity with method 2
  long currT_DC1 = micros();
  float deltaT_DC1 = ((float)(currT_DC1 - prevT_i_DC1)) / 1.0e6;
  velocity_i_DC1 = increment_DC1 / deltaT_DC1;
  prevT_i_DC1 = currT_DC1;
}

void readEncoder_DC2() {
  int b_DC2 = digitalRead(ENCB_DC2);
  int increment_DC2 = 0;
  if (b_DC2 > 0) {
    increment_DC2 = 1;
  } else {
    increment_DC2 = -1;
  }
  pos_i_DC2 = pos_i_DC2 + increment_DC2;

  //compute velocity with method 2
  long currT_DC2 = micros();
  float deltaT_DC2 = ((float)(currT_DC2 - prevT_i_DC2)) / 1.0e6;
  velocity_i_DC2 = increment_DC2 / deltaT_DC2;
  prevT_i_DC2 = currT_DC2;
}
