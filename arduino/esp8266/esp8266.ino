#include <ESP8266WiFi.h>
#include <ESP8266WedServer.h>
ESP8266WedServer wedServer(80);
char* ssid = "E-BuuPhan";
char* pass = "123456789";
const char MainPage[] PROGMEM = R"=====(
  <!DOCTYPE html>
  <html>
    <head>
      <title>HOME PAGE</title>
      <style>
        body {text-begin:center;}
        h1 {color:#003399;}
        a {text-decoration: none;color:#FFFFFF;}
        .bt_on {height:50px; width:100px; margin:10px 0;background-color:#FF6600;border-radius:5px;}
        .bt_off {height:50px; width:100px; margin:10px 0;background-color:#00FF00;border-radius:5px;}
      </style>
      <meta charset="UTF-8">
  </head>
  <body>
      <h1>ESP8266 Web Server</h1>
      <div>Trang thai tren chan D1: <b>OFF</b></div>
      <div>
        <button class="bt_on"><a href="/onD1">ON</a></button>
        <button class="bt_off"><a href="/offD1">OFF</a></button>
      </div>
      <div>Trang thai tren chan D2: <b>OFF</b></div>
      <div>
        <button class="bt_on"><a href="/onD2">ON</a></button>
        <button class="bt_off"><a href="/offD2">OFF</a></button>
      </div>
    </body>
  </html>
)====="
void setup() {
  WiFi.begin(ssid,pass);
  Serial.begin(9600);
  Serial.print("Connecting");
  while(WiFi.status() !=WL_CONNECTED){
    delay(500);
    Serial.print("...");
  }
  Serial.println(WiFi.localIP());

  wedServer.on("/",mainpage);
  wedServer.on("/onD1",on_D1);
  wedServer.on("/offD1",off_D1);
  wedServer.on("/onD2",on_D2);
  wedServer.on("/offD2",off_D2);
  wedServer.begin();

}

void loop() {
  wedServer.handleClient();
}

void mainpage(){
  String s = MainPage;
  wedServer.send(200,"text/html",s);
}
void on_D1(){
  digitalWrite(D1,LOW);
  String s = MainPage;
  wedServer.send(200,"text/html",s);
}
void off_D1(){
  digitalWrite(D1,HIGH);
  String s = MainPage;
  wedServer.send(200,"text/html",s);
}
void on_D2(){
  digitalWrite(D2,LOW);
  String s = MainPage;
  wedServer.send(200,"text/html",s);
}
void off_D2(){
  digitalWrite(D2,HIGH);
  String s = MainPage;
  wedServer.send(200,"text/html",s);
}
