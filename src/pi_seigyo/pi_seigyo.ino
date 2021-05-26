#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

void setup(void)
{
  Serial.begin(115200);
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
}

float r_pgain = 7;
float l_pgain = 7;
float r_igain = 0.09; //I制御
float l_igain = 0.09; //I制御
float integ = 0; //I制御
int goal = 0;

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  int gyro_x = euler.x();
  Serial.print("0,360,180,");
  Serial.println(euler.x());
  goal = 180 - gyro_x;
  integ = integ + goal;
  if (abs(goal) < 1)integ = 0;
  float vl = -1 * goal * l_pgain - 1 * integ * l_igain; //I制御(integ追加)
  float vr = goal * r_pgain + 1 * integ * r_igain; //I制御(integ追加)

  vr = min(max(vr, -255), 255); //PWMが255を超えないようにする
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
  delay(10); //微積分が早すぎるのでdelay
}
