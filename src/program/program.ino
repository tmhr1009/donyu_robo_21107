void free_moter(){
  digitalWrite(3, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(13, LOW);
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

void moter(int rot, int pwm){
  float pwm_f = pwm * 1.82;
  if(rot){
    analogWrite(3, LOW);
    digitalWrite(9, pwm_f);
    analogWrite(10, pwm);
    digitalWrite(11, LOW);
  }else{
    analogWrite(3, pwm_f);
    digitalWrite(9, LOW);
    analogWrite(10, LOW);
    digitalWrite(11, pwm);
  }
  return 0;
}

void setup() {
  pinMode(3, OUTPUT); //D3ピンを出力に設定
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop(){
  moter(0, 100);
  delay(5000);
  //moter(1, 150);
  //delay(2000);

  brake_moter();
  delay(2000);
  
}
