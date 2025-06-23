#include <Servo.h>

Servo myservo;
int val;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(9);
  Serial.flush();
}

void loop() {
  Serial.println("Enter Servo value: ");
  // put your main code here, to run repeatedly:
  while (Serial.available() == 0);
  {}
  val = Serial.parseInt();
  Serial.println("Servo Position: ");
  Serial.println(val);
  myservo.write(val);
  delay(15);
}

