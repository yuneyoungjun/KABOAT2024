#include<Servo.h> //Servo 라이브러리를 추가
Servo servo;      //Servo 클래스로 servo객체 생성

void setup() {
  servo.attach(7); 
}

void loop() {
    // range : 45 to 135

    int i = 45, mode = 0;
    while(1){
      if(i <= 45)       mode = 0;
      else if(i >= 135) mode = 1;
      
      if(!mode) i++;
      else i--;
      
      servo.write(i);
      delay(10);
    }
}