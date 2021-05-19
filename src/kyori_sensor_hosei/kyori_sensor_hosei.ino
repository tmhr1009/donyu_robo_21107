#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}


void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int gyro_x = euler.x();
  Serial.println(euler.x());

  int s_left = analogRead(A2);
  int s_center = analogRead(A1);
  int s_right = analogRead(A0);
  int hosei = 0;

  if (s_left > 400 || s_center > 20000 || s_right > 400) {
    hosei = 0;
    kaiten(s_left, s_center, s_right);
    Serial.println("kaiten");
  } else {
    hosei = 1;
  }

  Serial.print(s_left);
  Serial.print(", ");
  Serial.print(s_center);
  Serial.print(", ");
  Serial.println(s_right);
  delay(500);

  if (hosei == 1) {
    //通常走行方向補正
    if (gyro_x < 359.50 && gyro_x > 180) {
      cw(0, 180);
    } else if (gyro_x > 0.50 && gyro_x < 180) {
      ccw(0, 180);
    } else {
      moter(0, 180);
    }
  }

}

void moter(int rot, int pwm) {
  int pwm_l = pwm + 30;

  if (rot) {
    digitalWrite(9, LOW);
    analogWrite(3, pwm_l);
    analogWrite(10, pwm);
    digitalWrite(11, LOW);
  } else {
    analogWrite(9, pwm_l);
    digitalWrite(3, LOW);
    digitalWrite(11, LOW);
    analogWrite(10, pwm);
  }
  return 0;
}

void cw(int rot, int pwm) {

  int pwm_r = pwm + 50;

  if (rot) {
    digitalWrite(9, LOW);
    analogWrite(3, pwm);
    analogWrite(11, pwm_r);
    digitalWrite(10, LOW);
  } else {
    analogWrite(9, pwm);
    digitalWrite(3, LOW);
    digitalWrite(11, LOW);
    analogWrite(10, pwm_r);
  }
  return 0;
}

void ccw(int rot, int pwm) {

  int pwm_r = pwm + 50;

  if (rot) {
    digitalWrite(9, LOW);
    analogWrite(3, pwm_r);
    analogWrite(11, pwm);
    digitalWrite(10, LOW);
  } else {
    analogWrite(9, pwm_r);
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

void kaiten(int left, int center, int right) {
//  if (center < 200) {
//    if (left > 500) {
//      analogWrite(9, LOW);//100
//      digitalWrite(3, LOW);
//      digitalWrite(11, LOW);
//      analogWrite(10, LOW);
//      delay(300);
//    } else if (right > 500) {
//      analogWrite(9, LOW);
//      digitalWrite(3, LOW);
//      digitalWrite(11, LOW);
//      analogWrite(10, LOW);//100
//      delay(300);
//    } else {
//      analogWrite(9, LOW);
//      digitalWrite(3, LOW);
//      digitalWrite(11, LOW);
//      analogWrite(10, LOW);//100
//      delay(800);
//    }

  if (right > 400) {
    analogWrite(9, 100);//100
    digitalWrite(3, LOW);
    digitalWrite(11, LOW);
    analogWrite(10, 0);
    Serial.println("right");
  } else if (left > 400) {
    analogWrite(9, 0);
    digitalWrite(3, LOW);
    digitalWrite(11, LOW);
    analogWrite(10, 100);//100
    Serial.println("left");
  }

  analogWrite(9, LOW);
  digitalWrite(3, LOW); 
  digitalWrite(11, LOW);
  analogWrite(10, LOW);

  return 0;
}
