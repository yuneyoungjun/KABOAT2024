<h1>KABOAT2024</h1>
<h2> 2024.07.08</h2>
Arduino와 ROS연결<br>
rosserial으로 Arduino의 ROS 통신<br>
Python 에서 Publish한 데이터를 Arduino에서 받아와 Thruster, Servo Motor 작동



<h3>실행 방법</h3>

```bash
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

_port : 아두이노 포트 위치
_baud : 아두이노 baudrate
```