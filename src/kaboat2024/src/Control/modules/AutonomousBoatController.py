import numpy as np
from math import floor, ceil

class AutonomousBoatController:
    BOAT_WIDTH = 1.3
    AVOID_RANGE = 7
    MAX_RANGE = 5
    GAIN_PSI = 1
    GAIN_DISTANCE = 8


    def cost_func_angle(x):
        x = abs(x)
        return 0.01 * x if x <= 10 else 0.1 * (x - 10)

    def cost_func_distance(x):
        return 0 if x > 7 else 10 - 10/7 * x

    def calculate_safe_zone(ld):
        safe_zone = [AutonomousBoatController.AVOID_RANGE] * 360
        for i in range(-180, 181):
            if 0 < ld[i] < AutonomousBoatController.AVOID_RANGE:
                safe_zone[i] = 0

        temp = np.array(safe_zone)
        for i in range(-180, 180):
            if safe_zone[i] > safe_zone[i + 1]:
                for j in range(floor(i + 1 - np.arctan2(AutonomousBoatController.BOAT_WIDTH/2, ld[i + 1]) * 180 / np.pi), i + 1):
                    temp[j] = 0
            if safe_zone[i] < safe_zone[i + 1]:
                for j in range(i, ceil(i + np.arctan2(AutonomousBoatController.BOAT_WIDTH/2, ld[i]) * 180 / np.pi) + 1):
                    temp[j] = 0
        return temp

    def calculate_optimal_psi_d(ld, goal_psi):
        safe_zone = AutonomousBoatController.calculate_safe_zone(ld)
        theta_list = []

        for i in range(-180, 180):
            if safe_zone[i] > 0:
                cost = (AutonomousBoatController.GAIN_PSI * AutonomousBoatController.cost_func_angle(i - goal_psi) + 
                        AutonomousBoatController.GAIN_DISTANCE * AutonomousBoatController.cost_func_distance(ld[i]))
                theta_list.append([i, cost])
        return theta_list
        # return sorted(theta_list, key=lambda x: x[1])[0][0]
