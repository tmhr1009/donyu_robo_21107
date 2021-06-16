#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>

SoftwareSerial softSerial(7, 8); //RX, TX
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void setup(void)
{
  Serial.begin(115200);
  softSerial.begin(9600);
  softSerial.print("?");
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  MsTimer2::set(1, center_right);

}

float r_pgain = 7;
float l_pgain = 7;
float r_igain = 0.09;
float l_igain = 0.09;
float r_dgain = 0.15; //右モータのD(微分)ゲイン
float l_dgain = 0.15; //左モータのD(微分)ゲイン
float presabun = 0; //目標値までの度数を格納
float integ = 0;
int goal = 0;
int spd = 175; //まっすぐ進むときの速度
int sped = 0; //まっすぐ進むときの速度
int mokuhyou = 180; //目標度数
int counter1 = 0;

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  int gyro_x = euler.x();
  presabun = goal; //目標値までの度数を記憶
  goal = mokuhyou - gyro_x;
  integ = integ + goal;
  if (abs(goal) < 1)integ = 0; //目標値まで1度未満になったらinteg = 0

  //A0=Right, A1=Center, A2=Left
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);


  if (a0 > 700) softSerial.print("migi \r");
  if (a1 > 500) softSerial.print("zenpou \r");
  if (a2 > 550) softSerial.print("hidari \r");

  if (a1 > 500) {
    sped = spd * 0;
//    MsTimer2::start();
//    //softSerial.print("syougaibutu ari\r");
//    MsTimer2::stop();
  } else if (a1 > 300) {
    sped = spd * 0.5;
  } else {
    sped = spd;
  }


  //(goal - presabun) 目標値までの度数と1つ前の度数の差
  //(+ (goal - presabun) * l_dgain + (float)spd)追加
  float vl = -1 * goal * l_pgain - 1 * integ * l_igain + (goal - presabun) * l_dgain + (float)sped;

  //(- (goal - presabun) * r_dgain + (float)spd)追加
  float vr = goal * r_pgain + 1 * integ * r_igain - (goal - presabun) * r_dgain + (float)sped;

  vr = min(max(vr, -255), 255);
  vl = min(max(vl, -255), 255);

  Serial.print("vl   == ");
  Serial.print(vl);
  Serial.print("  vr  == ");
  Serial.println(vr);

  if (vr < 0) {
    digitalWrite(3, LOW);
    analogWrite(9, -1 * vr);
  }
  else {
    digitalWrite(9, LOW);
    analogWrite(3, vr);
  }

  if (vl < 0) {
    digitalWrite(11, LOW);
    analogWrite(10, -1 * vl);
  }
  else {
    digitalWrite(10, LOW);
    analogWrite(11, vl);
  }
  delay(20); //微積分が速すぎるのでdelay
}

void center_right() {
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);

  //右にも反応したとき
  if (a0 > 700) {
    if (a2 > 550) {
      if (mokuhyou == 0) {
        mokuhyou == 180;
      } else if (mokuhyou == 270) {
        mokuhyou = 90;
      } else {
        mokuhyou = mokuhyou - 90;
      }
    } else {
      if (mokuhyou == 0) {
        mokuhyou = 270;
      } else {
        mokuhyou = mokuhyou - 90;
      }
    }
  }
}
