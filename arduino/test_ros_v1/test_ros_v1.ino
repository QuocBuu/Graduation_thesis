#include <ros.h>
#include <geometry_msgs/Twist.h> 
#include <util/atomic.h>

  // Declare DC1
  #define PWM_DC1   13
  #define IN2_DC1   46
  // Declare DC2
  #define PWM_DC2   45
  #define IN2_DC2   44

  // Setting_boot_speed
  float vt_DC1;
  float vt_DC2;
  int speed_pwm = 80;
  int rotation_speed = 30;

  //
  int dir, pwr;
  
// Khai báo các biến toàn cục
double speed_ang;
double speed_lin;

void moveCallback(const geometry_msgs::Twist& msg) {
  speed_lin = msg.linear.x;   
  speed_ang = msg.angular.z;

  if (speed_ang > 0) {      // phải 
    pwr = speed_pwm;
    dir = 3;
  }
  if (speed_ang < 0) {      // trái
    pwr = speed_pwm;
    dir = 2;
  }
  if (speed_ang == 0) {
    if (speed_lin < 0) {    // tiến 
      pwr = speed_pwm;
      dir = -1;
    }
    if (speed_lin > 0) {    // lùi 
      pwr = speed_pwm;
      dir = 1;
    }
    if (speed_lin == 0) {   // dừng 
      pwr = 0;
    }
  }
}

// Khai báo các biến ROS
ros::NodeHandle nh;
geometry_msgs::Twist action;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &moveCallback);

void setup() {
  Serial.begin(9600);
  // Khởi tạo kết nối ROS
  nh.initNode();
  nh.subscribe(sub);

  pinMode(PWM_DC1,OUTPUT);
  pinMode(PWM_DC2,OUTPUT);
  pinMode(IN2_DC1,OUTPUT);
  pinMode(IN2_DC2,OUTPUT);
}

void loop() {
  setMotor(dir,pwr);
  setMotor(dir,pwr);
  // Cập nhật kết nối ROS
  nh.spinOnce();
}
void setMotor(int dir_1, int pwmVal) {
  analogWrite(PWM_DC1,pwmVal);
  analogWrite(PWM_DC2,pwmVal);
  if(dir_1 == 1){
    digitalWrite(IN2_DC1,LOW);
    digitalWrite(IN2_DC2,LOW);
  }
  if(dir_1 == -1){
    digitalWrite(IN2_DC1,HIGH);
    digitalWrite(IN2_DC2,HIGH);
  }
  if(dir_1 == 2){
    digitalWrite(IN2_DC1,LOW);
    digitalWrite(IN2_DC2,HIGH);
  }
  if(dir_1 == 3){
    digitalWrite(IN2_DC1,HIGH);
    digitalWrite(IN2_DC2,LOW);
  }
}
