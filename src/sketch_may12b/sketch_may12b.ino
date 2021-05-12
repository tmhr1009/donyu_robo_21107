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

  //moter(0, 0, gyro_x);


  if (gyro_x < 357 && gyro_x > 180) {
    ccw();
  } else if (gyro_x > 3 && gyro_x < 180) {
    cw();
  } else {
    brake_moter();
  }

  delay(100);
}

void moter(int rot, int pwm, int gyro_x) {
  int pwm_l = pwm, pwm_r = pwm;
  //  if (gyro_x > 2 && gyro_x < 180) {
  //    pwm_r = pwm -100;
  //  } else if (gyro_x < 359) {
  //    pwm_l = pwm -100;
  //  }


  if (rot) {
    analogWrite(3, LOW);
    digitalWrite(9, pwm_l);
    analogWrite(10, pwm_r);
    digitalWrite(11, LOW);
  } else {
    analogWrite(3, pwm_l);
    digitalWrite(9, LOW);
    analogWrite(10, LOW);
    digitalWrite(11, pwm_r);
  }
  return 0;
}

void cw() {
  digitalWrite(10, LOW);
  analogWrite(11, 100);
  return 0;
}


void ccw() {
  analogWrite(10, 100);
  digitalWrite(11, LOW);
  return 0;
}

void brake_moter(){
  digitalWrite(3, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(13, HIGH);
  return 0;
}
