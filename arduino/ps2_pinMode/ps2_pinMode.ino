#include <PS2X_lib.h>

PS2X ps2x;

void setup() {
  Serial.begin(9600);
  ps2x.config_gamepad(7, 6, 5, 8); // Cấu hình chân kết nối (CLK, CS, CMD, DAT)
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  ps2x.read_gamepad();

  if (ps2x.Button(PSB_PAD_UP)) { // Kiểm tra nút mũi tên lên
    Serial.println("Up");
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);    
  }
  else if (ps2x.Button(PSB_PAD_DOWN)) { // Kiểm tra nút mũi tên xuống
    Serial.println("Down");
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);  
  }
  else if (ps2x.Button(PSB_PAD_LEFT)) { // Kiểm tra nút mũi tên trái
    Serial.println("Left");
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
    digitalWrite(12, HIGH);
    digitalWrite(13, LOW);  
  }
  else if (ps2x.Button(PSB_PAD_RIGHT)) { // Kiểm tra nút mũi tên phải
    Serial.println("Right");
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
    digitalWrite(12, LOW);
    digitalWrite(13, HIGH);  
  }
  else if (ps2x.Button(PSB_BLUE)) { // Kiểm tra nút Xanh
    Serial.println("Blue");
  }
  else if (ps2x.Button(PSB_RED)) { // Kiểm tra nút Đỏ
    Serial.println("Red");
  }
  else if (ps2x.Button(PSB_GREEN)) { // Kiểm tra nút Xanh lá
    Serial.println("Green");
  }
  else if (ps2x.Button(PSB_PINK)) { // Kiểm tra nút Hồng
    Serial.println("Pink");
  }
  else {
    Serial.println("0");
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);  
  }

  // Các lệnh điều khiển khác tương tự

  // int xAxis = ps2x.Analog(PSS_LX);
  // int yAxis = ps2x.Analog(PSS_LY);
  // // LX 128, LY 127
  // if (xAxis >= 130) {
  //   if (yAxis >= 130) {
  //     digitalWrite(10, HIGH);
  //     digitalWrite(11, HIGH);
  //     digitalWrite(12, LOW);
  //     digitalWrite(13, LOW);
  //   }
  //   else if (yAxis <= 120) {
  //     digitalWrite(10, HIGH);
  //     digitalWrite(11, LOW);
  //     digitalWrite(12, LOW);
  //     digitalWrite(13, HIGH);
  //   }
  // }
  // else if (xAxis <= 120) {
  //   if (yAxis >= 130) {
  //     digitalWrite(10, LOW);
  //     digitalWrite(11, HIGH);
  //     digitalWrite(12, HIGH);
  //     digitalWrite(13, LOW);
  //   }
  //   else if (yAxis <= 120) {
  //     digitalWrite(10, LOW);
  //     digitalWrite(11, LOW);
  //     digitalWrite(12, HIGH);
  //     digitalWrite(13, HIGH);
  //   }
  // }
  // else {
  //   digitalWrite(10, LOW);
  //   digitalWrite(11, LOW);
  //   digitalWrite(12, LOW);
  //   digitalWrite(13, LOW);
  // }
}