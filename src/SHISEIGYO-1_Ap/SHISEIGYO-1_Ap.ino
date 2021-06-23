#include "MPU6886.h"
#include <Kalman.h>
#include "FastLED.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

WebServer server(80);

const char ssid[] = "SHISEIGYO-1";  // SSID
const char pass[] = "password";   // password

const IPAddress ip(192, 168, 22, 1);      // IPアドレス
const IPAddress subnet(255, 255, 255, 0); // サブネットマスク

#define ENC_A 22
#define ENC_B 19
#define brake 23
#define rote_pin 32
#define PWM_pin 26
#define button 39
#define led_pin 27
#define NUM_LEDS 25

CRGB leds[NUM_LEDS];
MPU6886 IMU;
int one[9] = {8, 1, 2, 3, 7, 12, 17, 18, 22};
int two[18] = {17, 0, 1, 2, 3, 4, 9, 10, 11, 12, 13, 14, 15, 20, 21, 22, 23, 24};
int three[18] = {17, 0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15, 20, 21, 22, 23, 24};

unsigned long oldTime = 0, loopTime, nowTime;
float dt;

volatile byte pos;
volatile int  enc_count = 0;

float Kp = 1.50;
float Kd = 1.53;
float Kw = 0.33;
float IDRS = 3.0;
float getupRange = 0.1;
float injectRatio = 15.25;
int rotMaxL = 760;
int rotMaxR = 750;

int DutyIni = 1007, pwmDuty;
int GetUP = 0;
int GetUpCnt = 0;

float M;
float Aj = 0.0;

int cnt, cntOld = 4;

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float temp = 0;

float theta_acc = 0.0;
float theta_dot = 0.0;

//オフセット
float accXoffset = 0, accYoffset = 0, accZoffset = 0;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

Kalman kalmanY;
float kalAngleY, kalAngleDotY;

Preferences preferences;

//センサオフセット算出
void offset_cal(){
  Aj = 0.0;
  
  for(int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  
  delay(1000);
  accXoffset = 0;
  accYoffset = 0;
  accZoffset = 0;
  gyroXoffset = 0;
  gyroYoffset = 0;
  gyroZoffset = 0;

  for(int i=0; i<10; i++) {
    IMU.getAccelData(&accX,&accY,&accZ);
    IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    delay(10);
    accXoffset += accX;
    accYoffset += accY;
    accZoffset += accZ;
    gyroXoffset += gyroX;
    gyroYoffset += gyroY;
    gyroZoffset += gyroZ;
  }

  if(accXoffset < 0){
    accXoffset = accXoffset / 10 + 1.0 / sqrt(2.0);
  }else{
    accXoffset = accXoffset / 10 - 1.0 / sqrt(2.0);
  }
  accYoffset /= 10;
  accZoffset = accZoffset / 10 + 1.0 / sqrt(2.0);
  gyroXoffset /= 10;
  gyroYoffset /= 10;
  gyroZoffset /= 10;

  for(int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

//加速度センサから傾きデータ取得 [deg]
float get_theta_acc() {
  IMU.getAccelData(&accX,&accY,&accZ);
  //傾斜角導出 単位はdeg
  theta_acc  = atan(-1.0 * (accX - accXoffset) / (accZ - accZoffset)) * 57.29578f;
  return theta_acc;
}

//Y軸 角速度取得
float get_gyro_data() {
  IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  theta_dot = gyroY - gyroYoffset;
  return theta_dot;
}

//起き上がり
void getup(){
  digitalWrite(brake, HIGH);
  int rotMax;
  //回転方向
  if(kalAngleY < 0.0){
    rotMax = rotMaxL;
    digitalWrite(rote_pin, LOW);
    GetUP = 1;
  }else{
    rotMax = rotMaxR;
    digitalWrite(rote_pin, HIGH);
    GetUP = 2;
  }

  for(int i = 1023; i >= rotMax; i--){
    ledcWrite(0, i);
    int cnt = map(i, 1023, rotMax, 3, 0);
    Serial.println(cnt);
    delay(5);

    //LEDインジケータ
    if(cnt != cntOld){
      for(int k = 0; k < NUM_LEDS; k++){
        leds[k] = CRGB::Black;
      }
      if(cnt == 1){
        for(int j = 0; j < one[0]; j++){
          leds[one[1+j]] = CRGB::Cyan;
        }
      }else if(cnt == 2){
        for(int j = 0; j < two[0]; j++){
          leds[two[1+j]] = CRGB::Magenta;
        }
      }else if(cnt == 3){
        for(int j = 0; j < three[0]; j++){
          leds[three[1+j]] = CRGB::Yellow;
        }
      }else if(cnt == 0){
        for(int k = 0; k < NUM_LEDS; k++){
          leds[k] = CRGB::White;
        }
      }
      FastLED.show();
    }
    cntOld = cnt;
  }
  ledcWrite(0, rotMax);
  delay(300);
  for(int k = 0; k < NUM_LEDS; k++){
    leds[k] = CRGB::Black;
  }
  FastLED.show();
  if(kalAngleY > 0.0){
    digitalWrite(rote_pin, LOW);
  }else{
    digitalWrite(rote_pin, HIGH);
  }
}

//ブラウザ表示
void handleRoot() {
  String temp ="<!DOCTYPE html> \n<html lang=\"ja\">";
  temp +="<head>";
  temp +="<meta charset=\"utf-8\">";
  temp +="<title>SHISEIGYO-1</title>";
  temp +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp +="<style>";
  temp +=".container{";
  temp +="  max-width: 500px;";
  temp +="  margin: auto;";
  temp +="  text-align: center;";
  temp +="  font-size: 1.2rem;";
  temp +="}";
  temp +="span,.pm{";
  temp +="  display: inline-block;";
  temp +="  border: 1px solid #ccc;";
  temp +="  width: 50px;";
  temp +="  height: 30px;";
  temp +="  vertical-align: middle;";
  temp +="  margin-bottom: 20px;";
  temp +="}";
  temp +="span{";
  temp +="  width: 120px;";
  temp +="}";
  temp +="button{";
  temp +="  width: 80px;";
  temp +="  height: 40px;";
  temp +="  font-weight: bold;";
  temp +="  margin-bottom: 30px;";
  temp +="}";
  temp +="</style>";
  temp +="</head>";
  
  temp +="<body>";
  temp +="<div class=\"container\">";
  temp +="<h3>SHISEIGYO-1</h3>";
  
  //起き上がりボタン
  temp +="<button type=\"button\" ><a href=\"/GetUp\">GetUp</a></button><br>";

  //Kp
  temp +="Kp<br>";
  temp +="<a class=\"pm\" href=\"/KpM\">-</a>";
  temp +="<span>" + String(Kp) + "</span>";
  temp +="<a class=\"pm\" href=\"/KpP\">+</a><br>";

  //Kd
  temp +="Kd<br>";
  temp +="<a class=\"pm\" href=\"/KdM\">-</a>";
  temp +="<span>" + String(Kd) + "</span>";
  temp +="<a class=\"pm\" href=\"/KdP\">+</a><br>";

  //Kw
  temp +="Kw<br>";
  temp +="<a class=\"pm\" href=\"/KwM\">-</a>";
  temp +="<span>" + String(Kw) + "</span>";
  temp +="<a class=\"pm\" href=\"/KwP\">+</a><br>";

  //Rot Max L
  temp +="Rot Max L<br>";
  temp +="<a class=\"pm\" href=\"/rotMaxLm\">-</a>";
  temp +="<span>" + String(rotMaxL) + "</span>";
  temp +="<a class=\"pm\" href=\"/rotMaxLp\">+</a><br>";

  //Rot Max R
  temp +="Rot Max R<br>";
  temp +="<a class=\"pm\" href=\"/rotMaxRm\">-</a>";
  temp +="<span>" + String(rotMaxR) + "</span>";
  temp +="<a class=\"pm\" href=\"/rotMaxRp\">+</a><br>";

  //injectRatio
  temp +="injectRatio<br>";
  temp +="<a class=\"pm\" href=\"/injectRatioP\">-</a>";
  temp +="<span>" + String(injectRatio) + "</span>";
  temp +="<a class=\"pm\" href=\"/injectRatioM\">+</a><br>";
 
  temp +="</div>";
  temp +="</body>";
  server.send(200, "text/HTML", temp);
}

void handleGetUp() {
  handleRoot();
  if(GetUP == 0){
    offset_cal();
    getup();
  }
}

void KpM() {
  if(Kp >= 0.01){
    Kp -= 0.01;
    preferences.putFloat("Kp", Kp);
  }
  handleRoot();
}

void KpP() {
  if(Kp <= 30){
    Kp += 0.01;
    preferences.putFloat("Kp", Kp);
  }
  handleRoot();
}

void injectRatioP(){
  if(injectRatio >= 0.01){
    injectRatio -= 0.01;
    preferences.putFloat("injectRatio", injectRatio);
  }
  handleRoot();
}

void injectRatioM(){
  if(injectRatio <= 30){
    injectRatio += 0.01;
    preferences.putFloat("injectRatio", injectRatio);
  }
  handleRoot();
}

void KdM() {
  if(Kd >= 0.01){
    Kd -= 0.01;
    preferences.putFloat("Kd", Kd);
  }
  handleRoot();
}

void KdP() {
  if(Kd <= 30){
    Kd += 0.01;
    preferences.putFloat("Kd", Kd);
  }
  handleRoot();
}

void KwM() {
  if(Kw >= 0.01){
    Kw -= 0.01;
    preferences.putFloat("Kw", Kw);
  }
  handleRoot();
}

void KwP() {
  if(Kw <= 30){
    Kw += 0.01;
    preferences.putFloat("Kw", Kw);
  }
  handleRoot();
}

void handleRotMaxLm() {
  if(rotMaxL >= 10){
    rotMaxL -= 10;
    preferences.putInt("rotMaxL", rotMaxL);
  }
  handleRoot();
}

void handleRotMaxLp() {
  if(rotMaxL <= 1010){
    rotMaxL += 10;
    preferences.putInt("rotMaxL", rotMaxL);
  }
  handleRoot();
}

void handleRotMaxRm() {
  if(rotMaxR >= 10){
    rotMaxR -= 10;
    preferences.putInt("rotMaxR", rotMaxR);
  }
  handleRoot();
}

void handleRotMaxRp() {
  if(rotMaxR <= 1010){
    rotMaxR += 10;
    preferences.putInt("rotMaxR", rotMaxR);
  }
  handleRoot();
}

void setup() {
  Serial.begin(115200);
  
  FastLED.addLeds<WS2812B, led_pin, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(20);
  
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(brake, OUTPUT);
  pinMode(button, INPUT);
 
  attachInterrupt(ENC_A, ENC_READ, CHANGE);
  attachInterrupt(ENC_B, ENC_READ, CHANGE);

  IMU.Init();

  //センサオフセット算出
  offset_cal();

  //フルスケールレンジ
  IMU.SetAccelFsr(IMU.AFS_2G);
  IMU.SetGyroFsr(IMU.GFS_250DPS);

  kalmanY.setAngle(get_theta_acc());

  ledcSetup(0, 20000, 10);
  ledcAttachPin(PWM_pin, 0);
  
  pinMode(rote_pin, OUTPUT);
  digitalWrite(brake, LOW);

  preferences.begin("parameter", false);

  //パラメータ初期値取得
  rotMaxL = preferences.getInt("rotMaxL", rotMaxL);
  rotMaxR = preferences.getInt("rotMaxR", rotMaxR);
  Kp = preferences.getFloat("Kp", Kp);
  Kd = preferences.getFloat("Kd", Kd);
  Kw = preferences.getFloat("Kw", Kw);
 

  WiFi.softAP(ssid, pass);           // SSIDとパスの設定
  delay(100);                        // 追記：このdelayを入れないと失敗する場合がある
  WiFi.softAPConfig(ip, ip, subnet); // IPアドレス、ゲートウェイ、サブネットマスクの設定
  
  IPAddress myIP = WiFi.softAPIP();  // WiFi.softAPIP()でWiFi起動

  server.on("/", handleRoot); 
  server.on("/GetUp", handleGetUp);
  
  server.on("/KpP", KpP);
  server.on("/KpM", KpM);
  server.on("/KdP", KdP);
  server.on("/KdM", KdM);
  server.on("/KwP", KwP);
  server.on("/KwM", KwM);
  server.on("/injectRatioP", injectRatioP);
  server.on("/injectRatioM", injectRatioM);
  
  server.on("/rotMaxLm", handleRotMaxLm);
  server.on("/rotMaxLp", handleRotMaxLp);
  server.on("/rotMaxRm", handleRotMaxRm);
  server.on("/rotMaxRp", handleRotMaxRp);
  server.begin();
}


void loop() {
  server.handleClient();
  
  //オフセット再計算&起き上がり
  if (digitalRead(button) == 0){
    Serial.println("オフセット再計算");
    offset_cal();
    getup();
  }
  
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  
  dt = (float)loopTime / 1000000.0; //sec
  
  //モータの角速度算出
  float theta_dotWheel = -1.0 * float(enc_count) * 3.6 / dt;
  enc_count = 0;
  
  //カルマンフィルタ 姿勢 傾き
  kalAngleY = kalmanY.getAngle(get_theta_acc(), get_gyro_data(), dt);
  
  //カルマンフィルタ 姿勢 角速度
  kalAngleDotY = kalmanY.getRate();

  if(GetUP == 1 || GetUP == 2){
    if(GetUP == 1 && kalAngleY >= getupRange){
      digitalWrite(brake, LOW);
      GetUP = 99;
    }else if(GetUP == 2 && kalAngleY <= -getupRange){
      digitalWrite(brake, LOW);
      GetUP = 99;
    }else{
      if(GetUP == 1 && kalAngleY < 0 || GetUP == 2 && kalAngleY > 0){
        ledcWrite(0, max(DutyIni - int(injectRatio * fabs(kalAngleY)), 0));
      }else {
        ledcWrite(0,511);
      }
    }
  }else {
    if (fabs(kalAngleY) < 1 && GetUP == 0){
      GetUP = 80;
    }

    /*
    Serial.print("Kp: ");
    Serial.print(Kp);
    Serial.print(", Kd: ");
    Serial.print(Kd,3);
    Serial.print(", Kw: ");
    Serial.print(Kw, 3);
    Serial.print(", DutyIni: ");
    Serial.print(DutyIni);
    Serial.print(", kalAngleY: ");
    Serial.print(kalAngleY);
    */
      
    
    if(GetUP == 99 || GetUP == 80){
      //ブレーキ
      if(fabs(kalAngleY) > 25.0){
        digitalWrite(brake, LOW);
        Aj = 0.0;
        GetUP = 0;
      }else{
        digitalWrite(brake, HIGH);
      }
        
      //モータ回転
      if(GetUP == 80){
        M = Kp * kalAngleY / 90.0 + Kd * kalAngleDotY / 500.0 + Kw * theta_dotWheel / 20000.0;
        GetUpCnt++;
        if(GetUpCnt > 100){
          GetUpCnt = 0;
          GetUP = 99;
        }
      }else{
        Aj +=  IDRS * theta_dotWheel / 1000000.0;
        M = Kp * (kalAngleY + Aj) / 90.0 + Kd * kalAngleDotY / 500.0 + Kw * theta_dotWheel / 10000.0;
      }
      M = max(-1.0f, min(1.0f, M));
      pwmDuty = DutyIni * (1.0 - fabs(M));
      
        
      //回転方向
      if(pwmDuty > DutyIni){
        digitalWrite(brake, LOW);
        ledcWrite(0, 1023);
      }else if(M > 0.0){
        digitalWrite(rote_pin, LOW);
        ledcWrite(0, pwmDuty);
      }else{
        digitalWrite(rote_pin, HIGH);
        ledcWrite(0, pwmDuty);
      }
    }
      
    
    Serial.print(", loopTime: ");
    Serial.print((float)loopTime / 1000.0);
    
    
    //LED表示
    for(int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB::Black;
    }
    
    if(kalAngleY > 20.0){
      for(int i = 0; i < 5; i++){
        leds[i * 5] = CRGB::Red;
      }
    }else if(kalAngleY <= 20.0 && kalAngleY > 12.0){
      for(int i = 0; i < 5; i++){
        leds[i * 5] = CRGB::Green;
      }
    }else if(kalAngleY <= 12.0 && kalAngleY > 4.0){
      for(int i = 0; i < 5; i++){
       leds[i * 5 + 1] = CRGB::Green;
      }
    }else if(kalAngleY <= 4.0 && kalAngleY > 1.0){
      for(int i = 0; i < 5; i++){
        leds[i * 5 + 2] = CRGB::Green;
      }
    }else if(abs(kalAngleY) <= 1.0){
      for(int i = 0; i < 5; i++){
        leds[i * 5 + 2] = CRGB::Blue;
      }
    }else if(kalAngleY >= -4.0 && kalAngleY < -1.0){
      for(int i = 0; i < 5; i++){
        leds[i * 5 + 2] = CRGB::Green;
      }
    }else if(kalAngleY >= -12.0 && kalAngleY < -4.0){
      for(int i = 0; i < 5; i++){
        leds[i * 5 + 3] = CRGB::Green;
      }
    }else if(kalAngleY >= -20.0 && kalAngleY < -12.0){
      for(int i = 0; i < 5; i++){
        leds[i * 5 + 4] = CRGB::Green;
      }
    }else if(kalAngleY < -20.0){
      for(int i = 0; i < 5; i++){
        leds[i * 5 + 4] = CRGB::Red;
      }
    }
    FastLED.show();
    
    delay(2);
      
    Serial.println("");
  }
}

//ブラシレスモータエンコーダ出力 割り込み処理
//参考：https://jumbleat.com/2016/12/17/encoder_1/
void ENC_READ() {
  byte cur = (!digitalRead(ENC_B) << 1) + !digitalRead(ENC_A);
  byte old = pos & B00000011;
  byte dir = (pos & B00110000) >> 4;
 
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
 
  if (cur != old)
  {
    if (dir == 0)
    {
      if (cur == 1 || cur == 3) dir = cur;
    } else {
      if (cur == 0)
      {
        if (dir == 1 && old == 3) enc_count--;
        else if (dir == 3 && old == 1) enc_count++;
        dir = 0;
      }
    }
 
    bool rote = 0;
    if (cur == 3 && old == 0) rote = 0;
    else if (cur == 0 && old == 3) rote = 1;
    else if (cur > old) rote = 1;
 
    pos = (dir << 4) + (old << 2) + cur;
  }
}
