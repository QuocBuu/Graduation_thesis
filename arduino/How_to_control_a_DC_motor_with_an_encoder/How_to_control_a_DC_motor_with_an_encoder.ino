#define ENCA 2 //YELLOW
#define ENCB 3 //WHITE
#define PWM 5
#define IN2 6

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int k;
void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  /*setMotor(1,25,PWM,IN1,IN2);
  delay(200);
  Serial.println(pos);
  setMotor(-1,25,PWM,IN1,IN2);
  delay(200);
  Serial.println(pos);
  setMotor(0,25,PWM,IN1,IN2);
  delay(200);
  Serial.println(pos);*/
  //set target position
  int target = 1200;

  //PID constants
  float kp = 1;
  float kd = 0;
  float ki = 0;

  //time difference
  long currT = micros();

  float deltaT = ((float)(currT - prevT))/1.0e6;
  prevT = currT;

  //error
  int e = pos - target;

  //derivative
  float dedt = (e-eprev)/(deltaT);

  //integral
  eintegral = eintegral + e*deltaT;

  //control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  //motor power
  float pwr = 100;

  //motor direction
  int dir = 1;


  //signal the motor
  setMotor(dir,pwr,PWM,IN2);

  //store previous error
  eprev = e;
encoder(pos);
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in2,LOW);
  }
}
void encoder( int pos)
{
  int k;
  delay(100);
  int q = pos - k;
  k = pos;
  Serial.println(q);
}
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }
  else{
    pos--;
  }
}
