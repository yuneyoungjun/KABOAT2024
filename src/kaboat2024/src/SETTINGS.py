import utm

""" 삼성중공업 """
ref_gps_x, ref_gps_y = 127.401794, 36.395991

""" 충남대학교 """
# ref_gps_x, ref_gps_y= 127.345411, 36.368706

ref_utm_x, ref_utm_y, _, _ = utm.from_latlon(ref_gps_y, ref_gps_x)

isSimulator = True
if(isSimulator==False):

    """
        Autonomous Parameters

        BOAT_WIDTH      배의 크기 (장애물 Padding)
        AVOID_RANGE     갈 수 없는 구역으로 만드는 장애물 거리
        GAIN_PSI        목적지와 배의 상대각도
        GAIN_DISTANCE   각도의 LaserScan값
    """
    BOAT_WIDTH = 0.3
    AVOID_RANGE = 2
    GAIN_PSI = 1
    GAIN_DISTANCE = 8
    GoalRange = 2

    """
        PID Control

        Kp              비례 계수
        Kd              미분 계수
        maxSaturation   모터 최대 출력
    """
    Kp = 10.0 
    Kd = 5.0 
    maxSaturation = 200

else:
    """ 
        시뮬레이터의 파라미터 값 
    """ 
    BOAT_WIDTH = 1.3
    AVOID_RANGE = 7
    GAIN_PSI = 1
    GAIN_DISTANCE = 8
    GoalRange = 2

    Kp = 20.0  # 비례 계수
    Kd = 5.0  # 미분 계수
    maxSaturation = 250