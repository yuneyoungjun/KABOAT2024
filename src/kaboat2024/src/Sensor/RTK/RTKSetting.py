import utm

# 삼성 중공업
# ref_gps_x = 127.401794
# ref_gps_y = 36.395991

# 충남대
ref_gps_x = 127.345411
ref_gps_y = 36.368706




ref_utm_x, ref_utm_y, _, _ = utm.from_latlon(ref_gps_y, ref_gps_x)