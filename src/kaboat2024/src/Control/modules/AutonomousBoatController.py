import numpy as np
from math import floor, ceil

class AutonomousBoatController:
    def __init__(self):
        # Constants and Settings
        self.BOAT_WIDTH = 1.2
        self.AVOID_RANGE = 15
        self.MAX_RANGE = 5
        self.GAIN_PSI = 1
        self.GAIN_DISTANCE = 8

        # Initialize boat state
        self.pos = [0, 0]
        self.psi = 0
        self.lidar_data = [0] * 360

    def update_data(self, lidar_data, psi, pos_x, pos_y):
        self.lidar_data = lidar_data
        self.psi = psi
        self.pos[0], self.pos[1] = pos_x, pos_y

    def cost_func_angle(self, x):
        x = abs(x)
        return 0.01 * x if x <= 10 else 0.1 * (x - 10)

    def cost_func_distance(self, x):
        return 0 if x > 7 else 10 - 10/7 * x

    def calculate_safe_zone(self, ld):
        safe_zone = [self.AVOID_RANGE] * 360
        for i in range(-180, 181):
            if 0 < ld[i] < self.AVOID_RANGE:
                safe_zone[i] = 0

        temp = np.array(safe_zone)
        for i in range(-180, 180):
            if safe_zone[i] > safe_zone[i + 1]:
                for j in range(floor(i + 1 - np.arctan2(self.BOAT_WIDTH/2, ld[i + 1]) * 180 / np.pi), i + 1):
                    temp[j] = 0
            if safe_zone[i] < safe_zone[i + 1]:
                for j in range(i, ceil(i + np.arctan2(self.BOAT_WIDTH/2, ld[i]) * 180 / np.pi) + 1):
                    temp[j] = 0
        return temp

    def calculate_optimal_psi_d(self, ld, goal_psi):
        safe_zone = self.calculate_safe_zone(ld)
        theta_list = []

        for i in range(-180, 181):
            if safe_zone[i] > 0:
                cost = (self.GAIN_PSI * self.cost_func_angle(i - goal_psi) + 
                        self.GAIN_DISTANCE * self.cost_func_distance(ld[i]))
                theta_list.append([i, cost])
        return theta_list
        # return sorted(theta_list, key=lambda x: x[1])[0][0]
