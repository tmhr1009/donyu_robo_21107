#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial softSerial(7, 8); //RX, TX

void setup(void)
{
  Serial.begin(9600);
  softSerial.begin(9600);
  softSerial.print("?");
}

int spd = 0;

void loop() {
  int a1 = analogRead(A1);
  //Serial.println(a1);

  if (a1 > 500) {
    brake_moter();
    softSerial.print("sutoppunau\r");
  } else if (a1 > 250) {
    moter(0, 175);
  } else {
    moter(0, 75);
  }

  delay(100);
}

void moter(int rot, int pwm) {
  if (rot) {
    digitalWrite(9, LOW);
    analogWrite(3, pwm);
    analogWrite(10, pwm);
    digitalWrite(11, LOW);
  } else {
    analogWrite(9, pwm);
    digitalWrite(3, LOW);
    digitalWrite(11, LOW);
    analogWrite(10, pwm);
  }
  return 0;
}

void brake_moter() {
  digitalWrite(3, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(13, HIGH);
  return 0;
}
