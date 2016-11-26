#include <Servo.h>

Servo servo1; Servo servo2; 


void setup() {

  pinMode(1,OUTPUT);
  servo1.attach(9); 
  servo2.attach(10); 
  Serial.begin(9600);
  Serial.println("Ready");

}

void loop() {

  static int v = 0;

  if ( Serial.available()) {
    char ch = Serial.read();

    switch(ch) {
      case '0'...'9':
        v = v * 10 + ch - '0';
        break;
      case 'p':
        servo1.write(v);
        v = 0;
        Serial.println(v);
        break;
      case 'r':
        servo2.write(v);
        v = 0;
        Serial.println(v);
        break;
    }
  }



} 
