#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimedAction.h>
#include <SoftwareSerial.h>

SoftwareSerial softSerial(7, 8); //RX, TX
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void center1();
TimedAction moterAction = TimedAction(800, center1);

void setup(void)
{
  Serial.begin(115200);
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  softSerial.begin(9600);
  softSerial.print("?");
  delay(100);
  softSerial.print("ok \r");
}

float r_pgain = 5; //7
float l_pgain = 5; //7
float r_igain = 0.10; //0.10
float l_igain = 0.10; //0.10
float r_dgain = 0.15; //右モータのD(微分)ゲイン 0.15
float l_dgain = 0.15; //左モータのD(微分)ゲイン 0.15
float presabun = 0; //目標値までの度数を格納
float integ = 0;
int goal = 0;
int spd = 125; //まっすぐ進むときの速度
int sped = 0; //まっすぐ進むときの速度
int mokuhyou = 180; //目標度数

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  if (mokuhyou == 0) {
    mokuhyou = 270;
  }
  if (mokuhyou == 360) {
    mokuhyou = 90;
  }

  int gyro_x = euler.x();
  presabun = goal; //目標値までの度数を記憶
  goal = mokuhyou - gyro_x;
  integ = integ + goal;
  if (abs(goal) < 1)integ = 0; //目標値まで1度未満になったら停止

  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);

  //A0=Right, A1=Center, A2=Left
  if (a0 > 600) softSerial.print("migi \r");
  if (a1 > 600) softSerial.print("mae \r");
  if (a2 > 600) softSerial.print("hidari \r");


  if (a1 > 600 || a0 > 600 || a2 > 600) {
    sped = spd * 0;
    moterAction.check();
  } else if (a1 > 300) {
    sped = spd * 0.7;
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
  delay(20); //微積分が早すぎるのでdelay
}

void center1() {
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);

  //いずれかのIRが反応
  //A0=Right, A1=Center, A2=Left
  if (a1 > 600 || a0 > 600 || a2 > 600) {

    //前反応
    if (a1 > 600) {
      if (a0 > 600) {
        //前と右に反応
        //左に90°回転
        mokuhyou = mokuhyou - 90;

      } else if (a2 > 600) {
        //前と左に反応
        //右に90°回転
        mokuhyou = mokuhyou + 90;

      }
      //前だけ反応
      //右に90°回転
      mokuhyou = mokuhyou + 90;

      //左反応
    } else if (a2 > 550) {
      if (a0 > 550) {
        //左と右に反応
        //なし
        mokuhyou = mokuhyou;

      }
      //左だけ反応
      //右に90°回転
      mokuhyou = mokuhyou + 90;

      //右反応
    } else if (a0 > 550) {
      //右のみ反応
      //左に90°回転
      mokuhyou = mokuhyou - 90;
    }
  }
}
