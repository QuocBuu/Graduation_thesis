#include <SoftwareSerial.h>
SoftwareSerial BLUETOOTH(A15, A14);
#include <util/atomic.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

// Khởi tạo đối tượng ROS Node
ros::NodeHandle nh;

// Phần cứng 
#define L_DC_DC   (0.4) // Khoảng cách giữa 2 động cơ (m)
#define R_DC      (0.1) // Bán kính bánh xe (m)
// Declare DC1
#define ENCA_DC1  2
#define ENCB_DC1  A1
#define PWM_DC1   13
#define IN2_DC1   46
// Declare DC2
#define ENCA_DC2  3
#define ENCB_DC2  49
#define PWM_DC2   45
#define IN2_DC2   44

char c;  //for receiving bluetooth
int a = 100;
int pwr_DC1;
long prevT_DC1 = 0;
int posPrev_DC1 = 0;
volatile int pos_i_DC1 = 0;
volatile float velocity_i_DC1 = 0;
volatile long prevT_i_DC1 = 0;
float eintegral_DC1 = 0;

int pwr_DC2;
long prevT_DC2 = 0;
int posPrev_DC2 = 0;
volatile int pos_i_DC2 = 0;z
volatile float velocity_i_DC2 = 0;
volatile long prevT_i_DC2 = 0;
float eintegral_DC2 = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

// Hàm callback sẽ được gọi mỗi khi nhận được tin nhắn mới từ topic "car_speed"
void carSpeedCallback(const std_msgs::Float32MultiArray& cmd_msg) {
  SetpointA = cmd_msg.data[0]; // Gán giá trị Setpoint tốc độ dài cho PID A
  SetpointB = cmd_msg.data[1]; // Gán giá trị Setpoint tốc độ góc cho PID B
}

// Khởi tạo đối tượng ROS Subscriber để đăng ký callback cho topic "car_speed"
ros::Subscriber<std_msgs::Float32MultiArray> sub("car_speed", carSpeedCallback);

void setup() {
  Serial.begin(9600);
  BLUETOOTH.begin(9600);
  //DC1
  pinMode(ENCA_DC1, INPUT);
  pinMode(ENCB_DC1, INPUT);
  pinMode(PWM_DC1, OUTPUT);
  pinMode(IN2_DC1, OUTPUT);
  //DC2
  pinMode(ENCA_DC2, INPUT);
  pinMode(ENCB_DC2, INPUT);
  pinMode(PWM_DC2, OUTPUT);
  pinMode(IN2_DC2, OUTPUT);

  // ROS
  // Khởi tạo đối tượng ROS Node  
  nh.initNode();
  // Đăng ký đối tượng ROS Subscriber
  nh.subscribe(sub);

  attachInterrupt(digitalPinToInterrupt(ENCA_DC1), readEncoder_DC1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_DC2), readEncoder_DC2, RISING);
}

void loop() {
  float vt_DC1;
  float vt_DC2;
  if (mode == 0) {
    //Dk Bluetooth
    if (BLUETOOTH.available()) {
      c = BLUETOOTH.read();
      Serial.write(c);
    }
    if (Serial.available()) {
      c = Serial.read();
      Serial.write(c);
      BLUETOOTH.write(c);
    }
    switch (c) {
      case '1':  // tiến
        a = 20;
        break;
      case '2':  // tiến
        a = 40;
        break;
      case '3':  // tiến
        a = 60;
        break;
      case '4':  // tiến
        a = 80;
        break;
      case '5':  // tiến
        a = 100;
        break;
      case '6':  // tiến
        a = 120;
        break;
      case '8':  // tiến
        a = 140;
        break;
      case '9':  // tiến
        a = 160;
        break;
      case 'F':  // tiến
        vt_DC1 = -a;
        vt_DC2 = a;
        break;
      case 'B':  // lùi
        vt_DC1 = a;
        vt_DC2 = -a;
        break;
      case 'R':  //sang phải
        vt_DC1 = a;
        vt_DC2 = a;
        break;
      case 'L':  // sang trái
        vt_DC1 = -a;
        vt_DC2 = -a;
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
  }
  else if (SetpointB =! 0) {
    vt_DC1 = (R_DC * Setpoint) * (1 - L_DC_DC/(2*R_DC));
    vt_DC2 = (R_DC * SetpointB) * (1 + L_DC_DC/(2*R_DC));
  }
  else {
    vt_DC1 = SetpointA;
    vt_DC2 = -SetpointA; 
  }

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

  //Compute velocity with method 2
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

  //set a target
  //float vt_DC1 = 100*sin(currT_DC1 /1e6);
  //float vt_DC2 = 100*sin(currT_DC1 /1e6);

  // compute the control signal u1
  float kp_DC1 = 2.2;
  float ki_DC1 = 4.2;
  // compute the control signal u2
  float kp_DC2 = 1.9;
  float ki_DC2 = 3.82;

  float e_DC1 = vt_DC1 - v1Filt;
  eintegral_DC1 = eintegral_DC1 + e_DC1 * deltaT_DC1;
  float u_DC1 = kp_DC1 * e_DC1 + ki_DC1 * eintegral_DC1;

  float e_DC2 = vt_DC2 - v2Filt;
  eintegral_DC2 = eintegral_DC2 + e_DC2 * deltaT_DC2;
  float u_DC2 = kp_DC2 * e_DC2 + ki_DC2 * eintegral_DC2;

  //Set the motor 1 speed and direction 
  int dir_DC1 = -1;
  if (u_DC1 < 0) {
    dir_DC1 = 1;
  }
  int pwr_DC1 = (int)fabs(u_DC1);
  if (pwr_DC1 > 255) {
    pwr_DC1 = 255;
  }
  //Set the motor 2 speed and direction
  int dir_DC2 = 1;
  if (u_DC2 < 0) {
    dir_DC2 = -1;
  }
  int pwr_DC2 = (int)fabs(u_DC2);
  if (pwr_DC2 > 255) {
    pwr_DC2 = 255;
  }

  // Điều khiển động cơ 
  setMotor(dir_DC1, pwr_DC1, PWM_DC1, IN2_DC1);
  setMotor(dir_DC2, pwr_DC2, PWM_DC2, IN2_DC2);

  // Gửi các giá trị encoder và tốc độ thực tế lên ROS
  std_msgs::Float32MultiArray enc_msg;
  enc_msg.data[0] = vt_DC1;
  enc_msg.data[1] = vt_DC2;
  nh.advertise("/car_encoder", enc_msg);

  // Lắng nghe các thông điệp ROS mới
  nh.spinOnce();

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
