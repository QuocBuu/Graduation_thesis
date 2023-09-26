#include <IRremote.h>
 int duration;
int RECV_PIN = 11;
int motor = 6;    // khai báo chân PWM điều khiển motor
int speedMotor = 0;     // biến lưu giữ tốc độ của motor
int s = 5;
int thuan;
IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  pinMode(motor,OUTPUT);
  pinMode(s,OUTPUT);
}

void loop() {
   
      duration = constrain(duration,0,5000);   // 0 <= duration <= 5000 ms.
      speedMotor = map(duration,0,5000,0,255);
      analogWrite(motor,speedMotor);  // xuất xung PWM để điều khiển tốc độ motor
                                      // duty cycle càng lớn thì motor quay càng nhanh
      delay(500);
      
  if (irrecv.decode(&results)) {
    Serial.println(results.value);
    if (results.value==16718055){
    digitalWrite(s, LOW);
    }
    if (results.value==16750695){
    duration = 0;
    }
    if (results.value==16730805){
    digitalWrite(s, HIGH);
    }
    if (results.value==16753245){
    duration = 1000;
    }
    if (results.value==16736925){
    duration = 3000;
    }
    if (results.value!=16769565){
    duration = 5000;
    }
    irrecv.resume(); // Receive the next value
    
  }
  
}
