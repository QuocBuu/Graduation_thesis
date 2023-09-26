int val;
int led1 = 2;
int led2 = 3;
int led3 = 4;
int led4 = 5;
int led5 = 6;
int led6 = 7;
int led7 = 8;
int led8 = 9;

void setup() {
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    pinMode(led4, OUTPUT);
    pinMode(led5, OUTPUT);
    pinMode(led6, OUTPUT);
    pinMode(led7, OUTPUT);
    pinMode(led8, OUTPUT);
    Serial.begin(9600);
}
void loop() {
delay(100);
if(Serial.available())
{
delay(50);
while(Serial.available() >0)
{

val=Serial.read();
Serial.println(val);
  if (val=='X'){
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
    digitalWrite(led5, LOW);
    digitalWrite(led6, LOW);
    digitalWrite(led7, LOW);
    digitalWrite(led8, LOW);
   }
   if (val=='Y'){
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);
    digitalWrite(led5, HIGH);
    digitalWrite(led6, HIGH);
    digitalWrite(led7, HIGH);
    digitalWrite(led8, HIGH);
   } 
  if (val=='A'){
    digitalWrite(led1, HIGH);
  }
  if (val=='B'){
    digitalWrite(led2, HIGH);
  }
  if (val=='C'){
    digitalWrite(led3, HIGH);
  }
  if (val=='D'){
    digitalWrite(led4, HIGH);
  }
  if (val=='E'){
    digitalWrite(led5, HIGH);
  }
  if (val=='F'){
    digitalWrite(led6, HIGH);
  }
  if (val=='G'){
    digitalWrite(led7, HIGH);
  }
  if (val=='H'){
    digitalWrite(led8, HIGH);
  }
  if (val=='K'){
    digitalWrite(led1, LOW);
  }
  if (val=='L'){
    digitalWrite(led2, LOW);
  }
  if (val=='M'){
    digitalWrite(led3, LOW);
  }
  if (val=='N'){
    digitalWrite(led4, LOW);
  }
  if (val=='O'){
    digitalWrite(led5, LOW);
  }
  if (val=='P'){
    digitalWrite(led6, LOW);
  }
  if (val=='Q'){
    digitalWrite(led7, LOW);
  }
  if (val=='R'){
    digitalWrite(led8, LOW);
  }

/*digitalWrite(led1, HIGH);
       digitalWrite(led2, LOW); 
       digitalWrite(led3, HIGH);
       digitalWrite(led4, LOW);
       digitalWrite(led5, HIGH);
       digitalWrite(led6, LOW);
       digitalWrite(led7, HIGH);
       digitalWrite(led8, LOW);
       delay(300);
       digitalWrite(led1, LOW);
       digitalWrite(led2, HIGH);
       digitalWrite(led3, LOW);
       digitalWrite(led4, HIGH);
       digitalWrite(led5, LOW);
       digitalWrite(led6, HIGH);
       digitalWrite(led7, LOW);
       digitalWrite(led8, HIGH);
       delay(300);
       digitalWrite(led1, HIGH);
       digitalWrite(led2, LOW); 
       digitalWrite(led3, HIGH);
       digitalWrite(led4, LOW);
       digitalWrite(led5, HIGH);
       digitalWrite(led6, LOW);
       digitalWrite(led7, HIGH);
       digitalWrite(led8, LOW);
       delay(300);
       digitalWrite(led1, LOW);
       digitalWrite(led2, HIGH);
       digitalWrite(led3, LOW);
       digitalWrite(led4, HIGH);
       digitalWrite(led5, LOW);
       digitalWrite(led6, HIGH);
       digitalWrite(led7, LOW);
       digitalWrite(led8, HIGH);
       delay(300);
              
       digitalWrite(led1, HIGH);
       digitalWrite(led2, HIGH); 
       digitalWrite(led3, HIGH);
       digitalWrite(led4, HIGH);
       digitalWrite(led5, HIGH);
       digitalWrite(led6, HIGH);
       digitalWrite(led7, HIGH);
       digitalWrite(led8, HIGH);
       delay(400);
       digitalWrite(led1, LOW);
       digitalWrite(led2, LOW);
       digitalWrite(led3, LOW);
       digitalWrite(led4, LOW);
       digitalWrite(led5, LOW);
       digitalWrite(led6, LOW);
       digitalWrite(led7, LOW);
       digitalWrite(led8, LOW);
       delay(400);
       
       digitalWrite(led1, HIGH);
       digitalWrite(led8, HIGH);
        delay(200);
       digitalWrite(led2, HIGH);
       digitalWrite(led7, HIGH);
        digitalWrite(led1, LOW);
       digitalWrite(led8, LOW);
        delay(200);
       digitalWrite(led3, HIGH);
       digitalWrite(led6, HIGH);
        digitalWrite(led2, LOW);
       digitalWrite(led7, LOW);
        delay(200);
       digitalWrite(led4, HIGH);
       digitalWrite(led5, HIGH);
        digitalWrite(led3, LOW);
       digitalWrite(led6, LOW);
        delay(200);
        digitalWrite(led4, LOW);
       digitalWrite(led5, LOW);
        delay(200);
         
       digitalWrite(led1, HIGH);
        delay(200);
        digitalWrite(led1, LOW);
       digitalWrite(led2, HIGH);
        delay(200);
        digitalWrite(led2, LOW);
       digitalWrite(led3, HIGH);
        delay(200);
         digitalWrite(led3, LOW);
       digitalWrite(led4, HIGH);
        delay(200);
         digitalWrite(led4, LOW);
       digitalWrite(led5, HIGH);
        delay(200);
         digitalWrite(led5, LOW);
        digitalWrite(led6, HIGH);
        delay(200);
         digitalWrite(led6, LOW);
        digitalWrite(led7, HIGH);
        delay(200);
         digitalWrite(led7, LOW);
        digitalWrite(led8, HIGH);
        delay(200);
         digitalWrite(led8, LOW);
       digitalWrite(led1, HIGH);
       digitalWrite(led2, LOW); 
       digitalWrite(led3, HIGH);
       digitalWrite(led4, LOW);
       digitalWrite(led5, HIGH);
       digitalWrite(led6, LOW);
       digitalWrite(led7, HIGH);
       digitalWrite(led8, LOW);
       delay(300);
       digitalWrite(led1, LOW);
       digitalWrite(led2, HIGH);
       digitalWrite(led3, LOW);
       digitalWrite(led4, HIGH);
       digitalWrite(led5, LOW);
       digitalWrite(led6, HIGH);
       digitalWrite(led7, LOW);
       digitalWrite(led8, HIGH);
       delay(300);
       digitalWrite(led1, HIGH);
       digitalWrite(led2, LOW); 
       digitalWrite(led3, HIGH);
       digitalWrite(led4, LOW);
       digitalWrite(led5, HIGH);
       digitalWrite(led6, LOW);
       digitalWrite(led7, HIGH);
       digitalWrite(led8, LOW);
       delay(300);
       digitalWrite(led1, LOW);
       digitalWrite(led2, HIGH);
       digitalWrite(led3, LOW);
       digitalWrite(led4, HIGH);
       digitalWrite(led5, LOW);
       digitalWrite(led6, HIGH);
       digitalWrite(led7, LOW);
       digitalWrite(led8, HIGH);
       delay(300);
              
       digitalWrite(led1, HIGH);
       digitalWrite(led2, HIGH); 
       digitalWrite(led3, HIGH);
       digitalWrite(led4, HIGH);
       digitalWrite(led5, HIGH);
       digitalWrite(led6, HIGH);
       digitalWrite(led7, HIGH);
       digitalWrite(led8, HIGH);
       delay(400);
       digitalWrite(led1, LOW);
       digitalWrite(led2, LOW);
       digitalWrite(led3, LOW);
       digitalWrite(led4, LOW);
       digitalWrite(led5, LOW);
       digitalWrite(led6, LOW);
       digitalWrite(led7, LOW);
       digitalWrite(led8, LOW);
       delay(400);
       
       digitalWrite(led1, HIGH);
       digitalWrite(led8, HIGH);
        delay(200);
       digitalWrite(led2, HIGH);
       digitalWrite(led7, HIGH);
        digitalWrite(led1, LOW);
       digitalWrite(led8, LOW);
        delay(200);
       digitalWrite(led3, HIGH);
       digitalWrite(led6, HIGH);
        digitalWrite(led2, LOW);
       digitalWrite(led7, LOW);
        delay(200);
       digitalWrite(led4, HIGH);
       digitalWrite(led5, HIGH);
        digitalWrite(led3, LOW);
       digitalWrite(led6, LOW);
        delay(200);
        digitalWrite(led4, LOW);
       digitalWrite(led5, LOW);
        delay(200);
         
       digitalWrite(led1, HIGH);
        delay(200);
        digitalWrite(led1, LOW);
       digitalWrite(led2, HIGH);
        delay(200);
        digitalWrite(led2, LOW);
       digitalWrite(led3, HIGH);
        delay(200);
         digitalWrite(led3, LOW);
       digitalWrite(led4, HIGH);
        delay(200);
         digitalWrite(led4, LOW);
       digitalWrite(led5, HIGH);
        delay(200);
         digitalWrite(led5, LOW);
        digitalWrite(led6, HIGH);
        delay(200);
         digitalWrite(led6, LOW);
        digitalWrite(led7, HIGH);
        delay(200);
         digitalWrite(led7, LOW);
        digitalWrite(led8, HIGH);
        delay(200);
         digitalWrite(led8, LOW); */

}
}
}
