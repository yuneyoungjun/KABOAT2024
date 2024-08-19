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

<h2> 2024.07.10</h2>
RADIOASTER TX12 조종기를 이용하여 Thruster 제어하기 

<h2> 2024.07.15</h2>
Manual Mode 원격 조종 실험

```
# ROS_MASTER_PC
~/.bashrc 파일에서
export ROS_IP="{MY_IPv4 Address}"
```

```
# ROS_REMOTE_PC
~/.bashrc 파일에서
export ROS_MASTER_URI=http://{MASTER_IP}:11311
```
공유기로 네트워크를 구성하여 remote PC에서 데이터를 주고 받을 수 있도록 하기

---
<h3>ROSBAG 사용법</h3>

```
#.bag 파일 만들기
rosbag record -a

#.bag 파일 실행
rosbag play {bag 파일 위치}
```
ROSBAG을 사용하여 실시간 데이터를 저장하여 세팅하는 데 사용


<h3>설치 패키지</h3>

```
src/livox_ros_driver2/
src/Livox-SDK2/
src/mapviz/
```