int motor = 6;    // khai báo chân PWM điều khiển motor
int speedMotor = 0;     // biến lưu giữ tốc độ của motor
int s = 3;
int thuan;
void setup() {
  Serial.begin(9600);   // giao tiếp Serial với 9600 baudrate.
  pinMode(motor,OUTPUT);
  pinMode(s,OUTPUT);
}

void loop() {
  digitalWrite(s, HIGH);
  int duration = 1200;   // đo thời gian duy trì trạng thái hiện tại
  duration = constrain(duration,0,5000);   // 0 <= duration <= 5000 ms.
      speedMotor = map(duration,0,5000,0,255);
    analogWrite(motor,speedMotor);  // xuất xung PWM để điều khiển tốc độ motor
                                      // duty cycle càng lớn thì motor quay càng nhanh
    delay(500);

}
