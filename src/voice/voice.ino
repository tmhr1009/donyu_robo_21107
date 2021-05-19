#include <SoftwareSerial.h>

SoftwareSerial softSerial(6,7); //RX, TX

void setup() {
  softSerial.begin(9600);
  softSerial.print("?");
}

void loop() {
  softSerial.print("a'ichi sougoukouka koutougakkou sennkouka\r");
  delay(3000);
//  while(1){
//  softSerial.print("ie'----i\r");
//  delay(2000);
//  }
}
