#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 12 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 11 (Arduino's Software TX)
SoftwareSerial XBee(12, 11); // RX, TX

// On initialise notre connection avec le XBee a 115200 bauds
void setup(){
  XBee.begin(115200);
  Serial.begin(9600);
  Serial.println("setup");
}

// Si on a des donnees sur le port serie, on les envoie au Xbee
void loop(){
 // If data comes in from Serial, send it out to Xbee
  int val = analogRead(A0);
  XBee.println(val);
  delay(500);
}