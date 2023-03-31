#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist,PoseArray,Pose
from sensor_msgs.msg import LaserScan


import dwaplanner

from threading import Lock,Thread
import time

def limitVal(minV,maxV,v):
    if v < minV:
        return minV
    if v > maxV:
        return maxV
    return v

class LocalPlanner:
    def __init__(self):
        self.arrive = 0.1
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vw = 0.0
        self.test_seq_count = 0
        # init plan_config for once
        self.laser_lock = Lock()
        self.plan_config = dwaplanner.Config(0.3)#0.1550000011920929
        c = self.plan_config
        self.threshold = c.max_speed*c.predict_time

        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.obs_pub = rospy.Publisher('/test/obs',PoseArray, queue_size=1)
        self.planner_thread = None

        self.need_exit = False

        self.updateMap()
        pass

    def updateMap(self):
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map',GetMap)
            msg = getMap().map
        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s'%e)
        # Update for planning algorithm
        self.mapCallback(msg)

    def mapCallback(self,msg):
        self.map = msg
        pass

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw
        ind = self.goal_index
        self.goal_index = len(self.path.poses)-1
        while ind < len(self.path.poses):
            p = self.path.poses[ind].pose.position
            dis = math.hypot(p.x-self.x,p.y-self.y)
            # print('mdgb;; ',len(self.path.poses),ind,dis)
            self.goal_index = ind
            if dis > self.threshold:
                break
            ind += 1
        goal = self.path.poses[self.goal_index]
        self.midpose_pub.publish(goal)
        lgoal = self.tf.transformPose("/robot_base", goal)
        self.plan_goal = np.array([lgoal.pose.position.x,lgoal.pose.position.y])
        self.goal_dis = math.hypot(self.x-self.path.poses[-1].pose.position.x,self.y-self.path.poses[-1].pose.position.y)

    def laserCallback(self,msg):
        # print("get laser msg!!!!",msg)
        self.laser_lock.acquire()
        # preprocess
        self.ob = [[100,100]]
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        for i in range(len(msg.ranges)):
            a = angle_min + angle_increment*i
            r = msg.ranges[i]
            if r < self.threshold:
                self.ob.append([math.cos(a)*r,math.sin(a)*r])
        self.laser_lock.release()
        pass

    def calculate(self):
        map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()
        ox,oy = np.nonzero(map_data > 50)#大于50是障碍物
        self.plan_ox = (ox*self.map.info.resolution+self.map.info.origin.position.x).tolist()
        self.plan_oy = (oy*self.map.info.resolution+self.map.info.origin.position.y).tolist()
        th = np.ones(len(self.plan_ox))
        obs = np.stack((self.plan_ox,self.plan_oy,th),axis=1)
        th = self.yaw
        R = np.array([[math.cos(th), math.sin(th), -self.x*math.cos(th)-self.y*math.sin(th)],[-math.sin(th), math.cos(th), self.x*math.sin(th)-self.y*math.cos(th)],[0,0,1]])
        now_obs = []
        for i in range(len(self.plan_ox)):
            now_obs.extend((np.matmul(R,np.transpose(obs[i,:]))))
        now_obs = np.array(now_obs)
        now_obs=now_obs.reshape(len(self.plan_ox),3)
        return now_obs[:,0:2]

    def updateObstacle(self):
        self.laser_lock.acquire()
        self.plan_ob = []
        self.plan_ob = np.array(self.ob)
        # self.plan_ob = self.calculate()
        # print(self.plan_ob)
        self.laser_lock.release()
        pass
    def pathCallback(self,msg):
        self.need_exit = True
        time.sleep(0.1)
        self.path = msg
        self.planner_thread = Thread(target=self.planThreadFunc)
        self.initPlanning()
        self.planner_thread.start()

    def initPlanning(self):
        self.goal_index = 0
        self.vx = 0.0
        self.vw = 0.0
        self.dis = 99999
        self.updateGlobalPose()
        cx = []
        cy = []
        for pose in self.path.poses:
            cx.append(pose.pose.position.x)
            cy.append(pose.pose.position.y)
        self.goal = np.array([cx[0],cy[0]])
        self.plan_cx,self.plan_cy = np.array(cx),np.array(cy)
        self.plan_goal = np.array([cx[-1],cy[-1]])
        self.plan_x = np.array([0.0,0.0,0.0,self.vx,self.vw])
        pass
    def planThreadFunc(self):
        print("running planning thread!!")
        self.need_exit = False
        while not self.need_exit:
            self.planOnce()
            self.publishTestObs(None)
            if self.goal_dis < self.arrive:
                print("arrive goal!")
                break
            time.sleep(0.001)
        print("exit planning thread!!")
        self.publishVel(True)
        self.planner_thread = None
        pass
    def planOnce(self):
        self.updateGlobalPose()
        # Update plan_x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.plan_x = [0.0,0.0,0.0,self.vx,self.vw]
        # Update obstacle
        self.updateObstacle()
        cal = dwaplanner.DWA()
        u= cal.plan(self.plan_x,self.plan_config,self.plan_goal,self.plan_ob)
        # print(u)
        alpha = 0.5
        self.vx = u[0]*alpha+self.vx*(1-alpha)
        self.vw = u[1]*alpha+self.vw*(1-alpha)
        # print(self.vx,self.vw)
        # print("mdbg; ",u)
        self.publishVel()
        pass
    def publishTestObs(self,obs):
        test_obs = PoseArray()
        test_obs.header.seq = self.test_seq_count
        test_obs.header.frame_id = "robot_base"
        test_obs.header.stamp = rospy.Time(0)

        for i in self.plan_ob:
            pose = Pose()
            pose.position.x = i[0]
            pose.position.y = i[1]
            pose.position.z = 0.01
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            test_obs.poses.append(pose)

        self.obs_pub.publish(test_obs)

        self.test_seq_count += 1
               
    def publishVel(self,zero = False):
        if zero:
            self.vx = 0
            self.vw = 0
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('path_Planning')
    lp = LocalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
