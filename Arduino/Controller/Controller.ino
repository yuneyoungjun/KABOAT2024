#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include<Servo.h>

Servo SL, SR, SF ;  //Servo Motor
Servo TL, TR, TF ;  //Thruster 

//tx12
int sensorPin[] = {0, 13, 12, 11, 10, 9, 8};
int channel[] = {0, 0, 0, 0, 0, 0, 0};


ros::NodeHandle  nh;
int data[] = {0, 0, 0, 0, 0, 0};

void messageCallback(const std_msgs::Int16MultiArray & sub_msg) {
  for(int i = 0; i < 6; i++)
  {
    data[i] = sub_msg.data[i];
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> sub("control", messageCallback); // 일단 보류

//servo 함수에서는 원하는 회전 각도를 입력하면 그 각도로 회전 (-45deg to 45deg)
void servo(int l = 0, int r = 0, int f = 0)
{
  l = constrain(l, -35, 35);
  r = constrain(r, -35, 35);
  f = constrain(f, -35, 35);

  SL.write(90 + l);
  SR.write(90 + r);
  SF.write(90 + f);
}

//servo 함수에서는 원하는 PWM값을 입력 (-500 to 500)
void thrust(int l = 0, int r = 0, int f = 0)
{
  l = constrain(l, -200, 200);
  r = constrain(r, -200, 200);
  f = constrain(f, -200, 200);

  TL.writeMicroseconds(1500+l); 
  TR.writeMicroseconds(1500+r); 
  TF.writeMicroseconds(1500+f); 
}

//Servo 객체와 선을 등록하고 기본상태로 초기화
void setting()
{
  SL.attach(2);
  SR.attach(3);
  SF.attach(4);
  
  SL.write(90);
  SR.write(90);
  SF.write(90);

  TL.attach(5);
  TR.attach(6);
  TF.attach(7);

  TL.writeMicroseconds(1500);
  TR.writeMicroseconds(1500);
  TF.writeMicroseconds(1500);
}


void tx12()
{
  for(int i = 1; i < 7; i++)
  {
    channel[i] = pulseIn(sensorPin[i],HIGH) - 1490;
    // channel[3] = 0;
    if(channel[i] < 30 && channel[i] > -30) channel[i] = 0;
    
    if (i == 3)
    {
      channel[i] = map(channel[i], -500, 500, -35, 35);
    }

    else
    {
      channel[i] = constrain(channel[i], -500, 500);
    }
  }
}

void setup() 
{
  Serial.begin(57600);
  setting();

//  nh.initNode();
//  nh.subscribe(sub);
}

void loop() 
{
  tx12();
  if(channel[5] < -100)
  {
    thrust(int(0.5 * (-channel[2]-channel[1])), // left
           int(0.5 * (-channel[2]+channel[1])), // right
           -channel[1]); // forward

    // servo(channel[3], channel[3], -channel[3]);
  }

  else
  {
    thrust(0, 0, 0);
  }
//  servo(data[0], data[1], data[2]);
//  thrust(data[3], data[4], data[5]);
//  nh.spinOnce();
}