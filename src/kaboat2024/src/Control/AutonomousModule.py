#!/usr/bin/env python3

import numpy as np
from math import ceil
from modules.AutonomousBoatController import AutonomousBoatController
from main import Boat


Goal_Psi = 0
Goal_Distance = 0

def normalize_angle(angle): return (angle + 180) % 360 - 180

def pathplan(boat=Boat(), goal_x=None, goal_y=None):
    global Goal_Psi, Goal_Distance

    if len(boat.scan) == 0:
        return [0, 0]

    safe_ld = AutonomousBoatController.calculate_safe_zone(boat.scan)
    cost_function = AutonomousBoatController.calculate_optimal_psi_d(safe_ld, int(Goal_Psi))
    psi_error = sorted(cost_function, key=lambda x: x[1])[0][0]
    psi_error = normalize_angle(psi_error)
    
    
    ## TauX를 계산하는 부분
    if goal_check():
        psi_error = Goal_Psi
        if psi_error < 30:
            tauX = min((Goal_Distance ** 3) + 120, 500)
        else:
            tauX_dist = min(3 * boat.scan[0] ** 2, 300)
            tauX_psi = 200 / (abs(psi_error) + 1)
            tauX = min(tauX_dist + tauX_psi, 500)
    else:
        tauX_dist = min(4 * Goal_Distance ** 2, 300)
        tauX_psi = 200 / (abs(psi_error) + 1)
        tauX = min(tauX_dist + tauX_psi, 500)

    # 목표 웨이포인트 데이터 표시
    if goal_x is not None and goal_y is not None:
        waypoint_x = goal_x - boat.position[0]
        waypoint_y = goal_y - boat.position[1]


        Goal_Psi = np.arctan2(waypoint_x, waypoint_y) * 180 / np.pi - boat.psi
        Goal_Psi = normalize_angle(Goal_Psi)
        Goal_Distance = np.sqrt(np.power(waypoint_x, 2) + np.power(waypoint_y, 2))

        return [psi_error, tauX]
    else:
        return [0, 0]


    

def goal_check(boat = Boat()):
    """목적지 까지 경로에 장애물이 있는지 판단하는 함수

    Args:
        Boat 객체

    Returns:
        장애물이 있는지 판단 결과를 리턴 [Boolean]
    """
    l = Goal_Distance
    theta = ceil(np.degrees(np.arctan2(AutonomousBoatController.BOAT_WIDTH/2, l)))

    check_ld = [0] * 360
    isAble = True

    for i in range(0, 90 - theta):
        angle = normalize_angle(int(Goal_Psi) - 90 + i)
        r = AutonomousBoatController.BOAT_WIDTH /(2 *np.cos(np.radians(i)))
        check_ld[angle] = r
        if(boat.scan[angle] == 0):
            continue
        if(r > boat.scan[angle]):
            isAble = False

    for i in range(-theta, theta + 1):
        check_ld[normalize_angle(int(Goal_Psi) + i)] = l
        if(boat.scan[normalize_angle(int(Goal_Psi) + i)] < l):
            isAble = False

    for i in range(0, 90 - theta):
        angle = normalize_angle(int(Goal_Psi) + 90 - i)
        r = AutonomousBoatController.BOAT_WIDTH /(2 *np.cos(np.radians(i)))
        check_ld[angle] = r
        if(boat.scan[angle] == 0):
            continue
        if(r > boat.scan[angle]):
            isAble = False

    return isAble

def goal_passed(boat = Boat(), goal_x = 0, goal_y = 0, goal_threshold = 2):
    """목적지에 도착했는지 판단하는 함수

    Args:
        Boat 객체
        목적지의 x, y 좌표

    Returns:
        도착 결과를 리턴 [Boolean]
    """
    isPassed = False
    if((boat.position[0] - goal_x)**2 + (boat.position[1] - goal_y)**2 < goal_threshold ** 2):
        isPassed = True
    return isPassed
