import utm

# 삼성 중공업
# ref_gps_x, ref_gps_y = 127.401794, 36.395991

# 충남대
ref_gps_x, ref_gps_y= 127.345411, 36.368706

ref_utm_x, ref_utm_y, _, _ = utm.from_latlon(ref_gps_y, ref_gps_x)


"""
    Autonomous Parameters
"""
BOAT_WIDTH = 1.3
AVOID_RANGE = 7
MAX_RANGE = 5
GAIN_PSI = 1
GAIN_DISTANCE = 8


