from heapq import heappush, heappop
from numpy import sqrt
import numpy as np
import math
from scipy.spatial import distance_matrix as dm

class Config:
    robot_radius = 0.35
    def __init__(self,obs_radius):
        self.obs_radius = obs_radius
        self.dt = 0.1  # [s] Time tick for motion prediction

        self.max_speed = 1 # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_accel = 1  # [m/ss]
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]

        self.max_yawrate = 100.0 * math.pi / 180.0  # [rad/s]
        self.max_dyawrate = 100.0 * math.pi / 180.0  # [rad/ss]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]

        
        self.predict_time = 0.5  # [s]

        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 0.1
        self.obstacle_cost_gain = 1.0

        self.tracking_dist = self.predict_time*self.max_speed
        self.arrive_dist = 0.1

class DWA:
    # heading保证机器人向目标点运动
    def heading_cost(self, from_x, from_y, to_x, to_y, robot_info_temp,):
        angleCost = np.arctan2((to_y - from_y),(to_x - from_x)) - robot_info_temp[2]
        now_dist = np.sqrt((to_x - from_x)**2 + (to_y - from_y)**2)
        return 2*np.abs(angleCost)+now_dist
        #+0.095 * np.abs(robot_info[4])
    
    # dist 保证机器人不碰到障碍物
    def dist_cost(self,dwaconfig, midpos,robot_info, planning_obs, radius):
        dist = 100000
        traj = []
        for i in range(int(dwaconfig.predict_time / dwaconfig.dt)):
            # for obs in planning_obs:
            #     now_dist = np.sqrt((obs[0] - robot_info[0])**2 + (obs[1] - robot_info[1])**2) - radius
            #     if now_dist < 0:
            #         return 100000
            #     else:
            #         dist = min(dist, now_dist)
            traj.append([robot_info[0],robot_info[1]])
            robot_info = self.motion(robot_info, dwaconfig)
        dist = dm(traj,planning_obs).min()
        return 1 / dist

    # velocity保证机器人尽量快
    def velocity_cost(self, now_v, max_v):
        delta=(max_v - now_v )
        return delta

    def motion(self, robot_info, dwaconfig):
        # 不考虑局部障碍物的运动
        new_robot_info = []
        new_robot_info.append(robot_info[0] + robot_info[3] * dwaconfig.dt * np.cos(robot_info[2] + robot_info[4] * dwaconfig.dt))
        new_robot_info.append(robot_info[1] + robot_info[3] * dwaconfig.dt * np.sin(robot_info[2] + robot_info[4] * dwaconfig.dt))
        new_robot_info.append(robot_info[2] + robot_info[4] * dwaconfig.dt)
        new_robot_info.append(robot_info[3])
        new_robot_info.append(robot_info[4])
        return new_robot_info
    #  robot info : [x, y, theta, vx, vw]
    #  dwaconfig : {robot_radius, obs_radius, dt, max_speed, min_speed, max_accel, v_reso, max_yawrate, max_dyawrate, yawrate_reso, predict_time, to_goal_cost_gain, speed_cost_gain, obstacle_cost_gain, tracking_dist, arrive_dist}
    def plan(self, robot_info, dwaconfig, midpos, planning_obs):
        # 返回结果
        nvx = 0.0
        nvw = 0.0
        all_u = []
        all_traj = []
        best_traj = []

        # 动态窗口初始化
        minV = max(dwaconfig.min_speed, robot_info[3] - dwaconfig.max_accel*dwaconfig.dt)
        maxV = min(dwaconfig.max_speed, robot_info[3] + dwaconfig.max_accel*dwaconfig.dt)
        minW = max(-dwaconfig.max_yawrate, robot_info[4] - dwaconfig.max_dyawrate*dwaconfig.dt)
        maxW = min(dwaconfig.max_yawrate, robot_info[4] + dwaconfig.max_dyawrate*dwaconfig.dt)
        possibleV_num = int((maxV - minV)/dwaconfig.v_reso) + 1
        possibleW_num = int((maxW - minW)/dwaconfig.yawrate_reso) + 1
        possibleV = np.linspace(minV, maxV, possibleV_num)
        possibleW = np.linspace(minW, maxW, possibleW_num)
        if dwaconfig.arrive_dist > np.sqrt((robot_info[0]-midpos[0])**2+(robot_info[1]-midpos[1])**2):
            return [0,0]#,best_traj,all_traj,all_u
        # time.sleep(1)
        # 遍历
        tot = 100000.0
        tot_info=robot_info
        cost = np.zeros((possibleV_num, possibleW_num))
        for vw in possibleW:
            for vx in possibleV:
                traj = []
                robot_info_temp = [robot_info[0], robot_info[1], robot_info[2], vx, vw]
                robot_info_temp = self.motion(robot_info_temp, dwaconfig)
                # 计算cost
                cost1 = self.heading_cost(robot_info_temp[0], robot_info_temp[1], midpos[0], midpos[1], robot_info_temp)
                cost2 = self.dist_cost(dwaconfig, midpos,robot_info_temp, planning_obs, 0.1)
                cost3 = self.velocity_cost(robot_info_temp[3], dwaconfig.max_speed)
                cost = cost1 * dwaconfig.to_goal_cost_gain + cost2 * dwaconfig.obstacle_cost_gain + cost3 * dwaconfig.speed_cost_gain
                # print(vx, vw, cost1, cost2, cost3, cost)
                if(cost < tot):
                    tot = cost
                    nvx = vx
                    nvw = vw
                    tot_info = robot_info_temp
                where = robot_info_temp
                for i in range(int(dwaconfig.predict_time / dwaconfig.dt)):
                    where = self.motion(where, dwaconfig)
                    traj.append([where[0],where[1]])
                all_traj.append(traj)
                all_u.append(cost)
        where = tot_info
        for i in range(int(dwaconfig.predict_time / dwaconfig.dt)):
            where = self.motion(where, dwaconfig)
            best_traj.append([where[0],where[1]])
        best_traj = np.array(best_traj)
        all_traj = np.array(all_traj)
        return [nvx, nvw]#,best_traj,all_traj,all_u
