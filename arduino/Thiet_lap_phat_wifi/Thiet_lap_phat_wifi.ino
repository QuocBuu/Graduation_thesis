#include <ESP8266WiFi.h>
char* ssip = "E-BuuPhan";
char* pass = "123456789";
IPAddress ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
void setup() {
  // put your setup code here, to run once:
WiFi.disconnect();
WiFi.mode(WIFI_AP);
WiFi.softAPConfig(ip, gateway, subnet);
WiFi.softAP(ssip, pass);

Serial.begin(9600);
Serial.println("ESP8266 operating in the access point mode!");
}

void loop() {


}
