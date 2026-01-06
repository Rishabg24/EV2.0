#include <Arduino.h>

#define ENA 6
#define IN1 7
#define IN2 8

#define ENB 9
#define IN3 10
#define IN4 11

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200);
  delay(1000);

  Serial.println("LEFT MOTOR FORWARD");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150);
  delay(2000);

  Serial.println("LEFT MOTOR STOP");
  analogWrite(ENA, 0);
  delay(1000);

  Serial.println("LEFT MOTOR REVERSE");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 150);
  delay(2000);

  Serial.println("LEFT MOTOR STOP");
  analogWrite(ENA, 0);
  delay(2000);


  Serial.println("RIGHT MOTOR FORWARD");
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 150);
  delay(2000);

  Serial.println("RIGHT MOTOR STOP");
  analogWrite(ENB, 0);
  delay(1000);

  Serial.println("RIGHT MOTOR REVERSE");
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 150);
  delay(2000);

  Serial.println("RIGHT MOTOR STOP");
  analogWrite(ENB, 0);

  Serial.println("TEST COMPLETE");
}

void loop() {}
