#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <stdio.h>

//ON and OFF
#define ON  1
#define OFF 0
// Button
#define button_1  2 //nút nhấn 1 - OFF/ON
#define button_2  3 //nút nhấn 2 - trái
#define button_3  4 //nút nhấn 3 - phải 
#define button_4  5 //nút nhấn 4 - điều khiển tốc độ nhấn thì 255
#define button_5  6 //nút nhấn 5 - điều khiển tốc độ nhấn thì 150
// Control motor
#define in1       8
#define in2       9
#define pwm       10
// Analog
#define control_signal A0 // tín hiệu điều khiển analog
// LCD 
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD 
// cái LCD tui để nguyên nghe không biết sử dụng loại 128*64 hay 64*24 
int speed_button_control = 0;

void setup() { 

  // Khai báo xung lúc dùng nhập số 115200 thì mượt hơn
  Serial.begin(9600);

  // khai báo GPIO: INPUT 
  pinMode(button_1,       INPUT);
  pinMode(button_2,       INPUT);
  pinMode(button_3,       INPUT);
  pinMode(button_4,       INPUT);
  pinMode(button_5,       INPUT);
  pinMode(control_signal, INPUT);

  // khai báo GPIO: OUTPUT
  pinMode(in1,            OUTPUT);
  pinMode(in2,            OUTPUT);
  pinMode(pwm,            OUTPUT);

   lcd.init(); //khởi tạo màn hình
   lcd.backlight(); //bật đèn màn hình
   lcd.begin(16,2);  
   lcd.setCursor(0,0);
   lcd.print("     WELCOME   ");
   lcd.setCursor(0,1);
   lcd.print("NGHIENCUUKHOAHOC");
   delay(3000); // Waiting for a while
   lcd.clear();
}

void loop() {
  // Read signal Button
  int ButtonRead_1 = digitalRead(button_1);
  int ButtonRead_2 = digitalRead(button_2);
  int ButtonRead_3 = digitalRead(button_3);
  int ButtonRead_4 = digitalRead(button_4);
  int ButtonRead_5 = digitalRead(button_5);

  // Read signal Res
  int Res_Read = analogRead(control_signal);
  int speed_res_control;

  if (ButtonRead_4 == 1) {
    speed_res_control = map(Res_Read, 0, 5000, 0, 255);
  } 
  else if (ButtonRead_5 == 1) {
    speed_res_control = map(Res_Read, 0, 5000, 0, 150);
  }
  else { speed_res_control = 0;}
  // không biết ông dùng biến trở gì nên không khớp thì thay đổi số  nghe 
  // map(tín hiệu đo, min_giá trị đo, max giá trị đo, min giá trị xuất, max giá trị xuất)

  
  if (ButtonRead_1 == ON && ButtonRead_2 == ON && ButtonRead_3 == OFF) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed_res_control);
  }
  else if (ButtonRead_1 == ON && ButtonRead_2 == OFF && ButtonRead_3 == ON) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, speed_res_control);
  }
  else {
    speed_res_control = 0;
    analogWrite(pwm, speed_res_control);
  }
  // lcd 
  lcd.setCursor(0,0);
  lcd.print("   Speed: ");
  lcd.print(Speed_LCD); 
  lcd.print("%d", speed_res_control);
}
