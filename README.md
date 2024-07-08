<h3> 2024.07.08</h3>
아두이노와 ROS연결<br>
ROS로 부터 Subscribe 받은 데이터를 아두이노에서 실행할 수 있게


<h5>실행 방법</h5>

roscore<br>
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

_port : 아두이노 포트 위치
_baud : 아두이노 baudrate
