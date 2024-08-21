import numpy as np
from math import floor, ceil

class AutonomousBoatController:
    def __init__(self):
        # Constants and Settings
        self.BOAT_WIDTH = 1.2
        self.END_RANGE = 1
        self.AVOID_RANGE = 15
        self.MAX_RANGE = 5
        self.GAIN_PSI = 1
        self.GAIN_DISTANCE = 8

        # Initialize boat state
        self.pos = [0, 0]
        self.psi = 0
        self.past_psi_d = 0
        self.lidar_data = [0] * 360

    def update_data(self, lidar_data, psi, pos_x, pos_y):
        self.lidar_data = lidar_data
        self.psi = psi
        self.pos[0], self.pos[1] = pos_x, pos_y

    def cost_func_angle(self, x):
        x = abs(x)
        return 0.01 * x if x <= 10 else 0.1 * (x - 10)

    def cost_func_distance(self, x):
        return 1 if x > 7 else 11 - 10/7 * x

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




    def Tau_X_calculate(self, ld,psi_error):
        safe_zone = self.calculate_safe_zone(ld)
        Tau_X=0
        Tau_X_cost = 0
        scale=0
        num=0
        if psi_error<0:
            scale=int(self.AVOID_RANGE)-int(min(psi_error,-self.AVOID_RANGE))
            for i in range(int(min(psi_error,-self.AVOID_RANGE)), int(self.AVOID_RANGE)):
                if safe_zone[i] > 0:
                    cost = (self.GAIN_DISTANCE * self.cost_func_distance(ld[i]))
                    Tau_X_cost+=cost
                else:
                    num+=1
                    
        else:
            scale=int(self.AVOID_RANGE)+int(max(psi_error,self.AVOID_RANGE))
            for i in range(int(-self.AVOID_RANGE), int(max(psi_error,self.AVOID_RANGE))):
                if safe_zone[i] > 0:
                    cost = (self.GAIN_DISTANCE * self.cost_func_distance(ld[i]))
                    Tau_X_cost+=cost
                else:
                    num+=1
        if num>abs(scale)/1.4:
            Tau_X/=(scale)/5
        else:
            Tau_X_cost*=8
            Tau_X_cost/=(scale+1)
            Tau_X=max(80,min(Tau_X_cost,500))
            print( Tau_X)

        return Tau_X
    



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

    def process_data(self, goal_x, goal_y):
        ld = np.array(self.lidar_data)

        goal_distance = np.sqrt((goal_x - self.pos[0])**2 + (goal_y - self.pos[1])**2)
        goal_psi = int(np.arctan2(goal_x - self.pos[0], goal_y - self.pos[1]) * 180 / np.pi - self.psi)
        goal_psi = (goal_psi + 180) % 360 - 180

        psi_d = self.calculate_optimal_psi_d(ld, goal_psi)

        if ld[goal_psi] > goal_distance:
            if all(ld[i] >= goal_distance for i in range(
                floor(goal_psi - np.arctan2(self.BOAT_WIDTH/2, ld[goal_psi]) * 180 / np.pi),
                ceil(goal_psi + np.arctan2(self.BOAT_WIDTH/2, ld[goal_psi]) * 180 / np.pi) + 1
            )):
                psi_d = goal_psi + 10000

        result = psi_d + self.psi if psi_d <= 9000 else psi_d - 10000 + self.psi
        self.past_psi_d = result

        if (goal_x - self.pos[0])**2 + (goal_y - self.pos[1])**2 < self.END_RANGE**2:
            result = -10000

        return result

# 사용 예시
if __name__ == "__main__":
    controller = AutonomousBoatController()
    
    # 데이터 업데이트 (실제로는 센서 데이터로 대체될 것입니다)
    lidar_data = [5] * 360  # 예시 LiDAR 데이터
    psi = 0  # 현재 선수각
    pos_x, pos_y = 0, 0  # 현재 위치
    
    controller.update_data(lidar_data, psi, pos_x, pos_y)
    
    # 목표 위치 설정
    goal_x, goal_y = 10, 10
    
    # 최적의 선수각 계산
    optimal_psi_d = controller.process_data(goal_x, goal_y)
    # print(f"Optimal Psi_d: {optimal_psi_d}")