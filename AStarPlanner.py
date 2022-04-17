from heapq import heappush, heappop
from numpy import sqrt
import numpy as np
class AStarPlanner:
    def __init__(self, unit_step = 0.1):
        self.unit_step = unit_step
    def planning(self, planning_obs_x, planning_obs_y, planning_obs_radius, planning_start_x, planning_start_y, planning_target_x, planning_target_y, planning_minx, planning_miny, planning_maxx, planning_maxy):
        heap = []
        # 离散化
        planning_start_x_int = int((planning_start_x - planning_minx) / self.unit_step)
        planning_start_y_int = int((planning_start_y - planning_miny) / self.unit_step)
        planning_target_x_int = int((planning_target_x - planning_minx) / self.unit_step)
        planning_target_y_int = int((planning_target_y - planning_miny) / self.unit_step)
        planning_max_x_int = int((planning_maxx - planning_minx) / self.unit_step)
        planning_max_y_int = int((planning_maxy - planning_miny) / self.unit_step)
        map = np.zeros((planning_max_x_int, planning_max_y_int))
        
        # 设置障碍物
        planning_obs_radius *= 1.0
        obs_radius_int = int(planning_obs_radius / self.unit_step)
        for i in range(planning_obs_x.shape[0]):
            obs_x_int = int((planning_obs_x[i] - planning_minx) / self.unit_step)
            obs_y_int = int((planning_obs_y[i] - planning_miny) / self.unit_step)
            for x in range(obs_x_int - obs_radius_int, obs_x_int + obs_radius_int):
                for y in range(obs_y_int - obs_radius_int, obs_y_int + obs_radius_int):
                    real_x = planning_minx + x * self.unit_step
                    real_y = planning_miny + y * self.unit_step
                    if x >= 0 and x < planning_max_x_int and y >= 0 and y < planning_max_y_int and (real_x - planning_obs_x[i]) ** 2 + (real_y - planning_obs_y[i]) ** 2 <= obs_radius_int ** 2:
                        map[x, y] = 1

        # 初始化
        heappush(heap, (0, planning_start_x_int, planning_start_y_int))
        came_from = {}
        cost_so_far = {}
        # came_from[(planning_start_x_int, planning_start_y_int)] = None
        cost_so_far[(planning_start_x_int, planning_start_y_int)] = 0
        while len(heap) > 0:
            current = heappop(heap)
            # 如果已经到达目标点，直接返回
            if current[1] == planning_target_x_int and current[2] == planning_target_y_int:
                break
            #只允许上下左右,斜向走
            for next in [(current[1] + 1, current[2]), (current[1] - 1, current[2]), (current[1], current[2] + 1), (current[1], current[2] - 1), (current[1] + 1, current[2] + 1), (current[1] - 1, current[2] - 1), (current[1] + 1, current[2] - 1), (current[1] - 1, current[2] + 1)]:
                # 若超出地图范围，则跳过
                if next[0] < 0 or next[0] >= planning_max_x_int or next[1] < 0 or next[1] >= planning_max_y_int:
                    continue
                # 换算next实际坐标
                next_x = next[0] * self.unit_step + planning_minx
                next_y = next[1] * self.unit_step + planning_miny
                # 若为障碍物，则跳过
                if map[next[0], next[1]] == 1:
                    continue
                # if ((next_x - planning_obs_x) ** 2 + (next_y - planning_obs_y) ** 2 <= planning_obs_radius ** 2).any():
                #     continue
                # for i in range(len(planning_obs_x)):
                #     if (next_x - planning_obs_x[i]) ** 2 + (next_y - planning_obs_y[i]) ** 2 < planning_obs_radius ** 2:
                #         continue
                # 换算current实际坐标
                current_x = current[1] * self.unit_step + planning_minx
                current_y = current[2] * self.unit_step + planning_miny
                # 计算新的cost,因为只允许向四个方向走，不加根号也可以!!!!!!!!!!!!!!!(加上了斜向走，所以不加根号不可以)
                new_cost = cost_so_far[(current[1], current[2])] + np.sqrt((next_x - current_x) ** 2 + (next_y - current_y) ** 2)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    #这里也忘了sqrt
                    priority = new_cost + sqrt((planning_target_x - next_x) ** 2 + (planning_target_y - next_y) ** 2)
                    heappush(heap, (priority, next[0], next[1]))
                    came_from[next] = (current[1], current[2])
        #如果到达不了
        if (planning_target_x_int, planning_target_y_int) not in cost_so_far:
            return [],[]
        #回溯路径，返回的是非离散空间的路径
        current = (planning_target_x_int, planning_target_y_int)
        path = []
        while current in came_from:
            path.append((current[0] * self.unit_step + planning_minx, current[1] * self.unit_step + planning_miny))
            current = came_from[current]
        path.append((planning_start_x, planning_start_y))
        path.reverse()
        pathnp = np.array(path)
        return pathnp[:,0], pathnp[:,1]