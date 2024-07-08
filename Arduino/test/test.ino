#include<Servo.h> //Servo 라이브러리를 추가
Servo servo1, servo2, servo3;      //Servo 클래스로 servo객체 생성

void setup() {
//  servo1.attach(7); 
//  servo2.attach(8); 
  servo3.attach(9); 
}

void loop() {
    // range : 45 to 135

    int i = 45, mode = 0;
    while(1){
      if(i <= 45)       mode = 0;
      else if(i >= 135) mode = 1;
      
      if(!mode) i++;
      else i--;
      
//      servo1.write(i);
//      servo2.write(i);
      servo3.write(i);
      delay(10);
    }
}
