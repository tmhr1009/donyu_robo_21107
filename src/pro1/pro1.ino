void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT); //D3ピンを出力に設定
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(3, LOW); //モータがfreeになる
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  delay(1000);//3000ms=3秒待機
  for (int i = 0; i < 250; i++) {
    analogWrite(3, i); //前進(のつもり
    digitalWrite(9, LOW);
    analogWrite(10, i);
    digitalWrite(11, LOW);
    delay(10);
  }
  for (int i = 250; i > 0; i--) {
    analogWrite(3, i); //前進(のつもり
    digitalWrite(9, LOW);
    analogWrite(10, i);
    digitalWrite(11, LOW);
    delay(10);
  }
}
