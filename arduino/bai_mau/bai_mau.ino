#include <PS2X_lib.h>
#include<util/atomic.h>
#define PS2_DAT        13  //14
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        12  //17
#define pressures    false        
#define rumble       false
PS2X ps2x;
int error = 0; byte type = 0;byte vibrate = 0;
// gan bien cho nut PS2
int A;
char c;
// Pins motor1
#define DC1ENCA 3
#define DC1ENCB 2
#define DC1PWM 4
#define DC1IN1 5
#define DC1IN2 6

#define PULSEPERROUND 2970

// Pins motor2
#define DC2ENCA 18
#define DC2ENCB 19
#define DC2PWM 7
#define DC2IN1 8
#define DC2IN2 9


// globals
long prevT_dc1 = 0;
int posPrev_dc1 = 0;
long prevT_dc2 = 0;
int posPrev_dc2 = 0;
volatile int pos_i_dc1= 0;
volatile int pos_i_dc2= 0;
//volatile float velocity_i_dc1 = 0;
//volatile float velocity_i_dc2 = 0;
float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral1 = 0;
float eintegral2 = 0;
float kp1 =10;
float ki1 = 20;

float kp2 =20;
float ki2 =40;


int pwr;
int a;
int dir;

void setup() {
  Serial.begin(115200);
  delay(300);//added delay to give wireless ps2 module
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
//MOTOR1
  pinMode(DC1ENCA,INPUT);
  pinMode(DC1ENCB,INPUT);
  pinMode(DC1PWM,OUTPUT);
  pinMode(DC1IN1,OUTPUT);
  pinMode(DC1IN2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(DC1ENCA),
                  readEncoderDC1,RISING);
//MOTOR2
  pinMode(DC2ENCA,INPUT);
  pinMode(DC2ENCB,INPUT);
  pinMode(DC2PWM,OUTPUT);
  pinMode(DC2IN1,OUTPUT);
  pinMode(DC2IN2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(DC2ENCA),
                  readEncoderDC2,RISING);
  
                 
}

void loop() {

  // read the position and velocity
  int pos1 = 0;
  int pos2 = 0;
  //float vel_at_1 = 0;
  //float vel_at_2 = 0;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos1 = pos_i_dc1;
    pos2 = pos_i_dc2;
    //vel_at_1 = velocity_i_dc1;
    //vel_at_2 = velocity_i_dc2;
  }
//Compute velocity with method 1
  long currT1 = micros();
  float deltaT1 = ((float)(currT1 - prevT_dc1))/1.0e6;
  float velocity1 = (pos1 - posPrev_dc1)/deltaT1;
  posPrev_dc1 = pos1;
  prevT_dc1 = currT1;

//Compute velocity with method 1
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
  float vt1= 25;
  float vt2= 25;
// compute the control signal u
  float e1 = vt1 - v1Filt;
  float e2 = vt2 - v2Filt;
  eintegral1 = eintegral1 + e1*deltaT1;
  eintegral2 = eintegral2 + e2*deltaT2;
  float u1 = kp1*e1 + ki1*eintegral1;
  float u2 = kp2*e2 + ki2*eintegral2;

  int pwr1 = (int) fabs(u1);
  int pwr2 = (int) fabs(u2);

  chuong_trinhPS2();
   
  if (A==1){chay_tien(pwr1,pwr2);} 
  if (A==2){chay_lui(pwr1,pwr2);}   
  if (A==3){re_phai(pwr1,pwr2);}
  if (A==4){re_trai(pwr1,pwr2);}
  if (A==5){xoay_trai(pwr1,pwr2);}
  if (A==6){xoay_phai(pwr1,pwr2);}
  if (A==0){reset(pwr1,pwr2);} 


  Serial.print(v1Filt);
  Serial.print(" ");  
  Serial.print(v2Filt);
  Serial.print(" ");    

  Serial.print(vt1); 
  Serial.println();
}


void chuong_trinhPS2()
{
//DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at ' vibrate' speed 
//CHAY TIEN
    if(ps2x.Button(PSB_PAD_UP)) //nut R2
    {A=1;    Serial.print("tien: ");}
    
//CHAY LUI
    else if(ps2x.Button(PSB_PAD_DOWN)) //nut R2
         {A=2;    Serial.print("lui: ");}

//RE PHAI
         else if(ps2x.Button(PSB_PAD_RIGHT)) //nut R2
              {A=3;    Serial.print("re phai: ");}

//RE TRAI
              else if(ps2x.Button(PSB_PAD_LEFT)) //nut R2
                   {A=4;    Serial.print("re trai: ");}

//XOAY TRAI
                   else if(ps2x.Button(PSB_L1)) //nut R2
                        {A=5;    Serial.print("xoay: ");}

//XOAY PHAI
                        else if(ps2x.Button(PSB_R1)) //nut R2
                             {A=6;    Serial.print("xoay: ");}
                             else {A=0;}


//delay(50);


     
}
void chay_tien(int a1, int a2)
{  

  //motor A
  digitalWrite(DC1IN1,HIGH);
  digitalWrite(DC1IN2,LOW);
  analogWrite(DC1PWM,a1); 
  //motor B
  digitalWrite(DC2IN1,HIGH);
  digitalWrite(DC2IN2,LOW);
  analogWrite(DC2PWM,a2); 
}

void chay_lui(int a1, int a2)
{  

  //motor A
  digitalWrite(DC1IN1,LOW);
  digitalWrite(DC1IN2,HIGH);
  analogWrite(DC1PWM,a1); 
  //motor B
  digitalWrite(DC2IN1,LOW);
  digitalWrite(DC2IN2,HIGH);
  analogWrite(DC2PWM,a2); 
}

void re_phai(int a1, int a2)

{  
  //motor A
  digitalWrite(DC1IN1,LOW);
  digitalWrite(DC1IN2,LOW);
  analogWrite(DC1PWM,a1); 
  //motor B
  digitalWrite(DC2IN1,HIGH);
  digitalWrite(DC2IN2,LOW);
  analogWrite(DC2PWM,a2);
}

void re_trai(int a1, int a2)

{  
  //motor A
  digitalWrite(DC1IN1,HIGH);
  digitalWrite(DC1IN2,LOW);
  analogWrite(DC1PWM,a1); 
  //motor B
  digitalWrite(DC2IN1,LOW);
  digitalWrite(DC2IN2,LOW);
  analogWrite(DC2PWM,a2);
}

void xoay_trai(int a1, int a2)

{  
  //motor A
  digitalWrite(DC1IN1,HIGH);
  digitalWrite(DC1IN2,LOW);
  analogWrite(DC1PWM,a1); 
  //motor B
  digitalWrite(DC2IN1,LOW);
  digitalWrite(DC2IN2,HIGH);
  analogWrite(DC2PWM,a2);
}

void xoay_phai(int a1, int a2)

{  
  //motor A
  digitalWrite(DC1IN1,LOW);
  digitalWrite(DC1IN2,HIGH);
  analogWrite(DC1PWM,a1); 
  //motor B
  digitalWrite(DC2IN1,HIGH);
  digitalWrite(DC2IN2,LOW);
  analogWrite(DC2PWM,a2);
}

void reset(int a1, int a2)
{  

  //motor A
  digitalWrite(DC1IN1,LOW);
  digitalWrite(DC1IN2,LOW);
  analogWrite(DC1PWM,255); 
  //motor B
  digitalWrite(DC2IN1,LOW);
  digitalWrite(DC2IN2,LOW);
  analogWrite(DC2PWM,255); 
}

void readEncoderDC1(){
  // Read encoder B when ENCA rises
  int b = digitalRead(DC1ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i_dc1 = pos_i_dc1 + increment;

}
void readEncoderDC2(){
  // Read encoder B when ENCA rises
  int b = digitalRead(DC2ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i_dc2 = pos_i_dc2 + increment;

}
