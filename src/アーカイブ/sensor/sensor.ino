void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  int s_left = analogRead(A2);
  int s_center = analogRead(A1);
  int s_right = analogRead(A0);

  Serial.print(s_left);
  Serial.print(", ");
  Serial.print(s_center);
  Serial.print(", ");
  Serial.println(s_right);

  
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
