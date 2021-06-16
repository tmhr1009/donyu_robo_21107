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
  float gyro_x = euler.x();
  Serial.println(euler.x());

  //moter(0, 0, gyro_x);

  //方向補正
  if (gyro_x < 358 && gyro_x > 180) {
    ccw();
  } else if (gyro_x > 2 && gyro_x < 180) {
    cw();
  } else {
    brake_moter();
  }

  //delay(50);
}

void cw() {
  digitalWrite(3, LOW);
  analogWrite(9, LOW);
  digitalWrite(10, LOW);
  analogWrite(11, 100);
  return 0;
}


void ccw() {
  analogWrite(3, LOW);
  digitalWrite(9, LOW);
  analogWrite(10, 100);
  digitalWrite(11, LOW);
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
