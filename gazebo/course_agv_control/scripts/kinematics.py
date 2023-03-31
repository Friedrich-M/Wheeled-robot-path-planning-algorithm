#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from std_msgs.msg import Float64

class Kinematics:
    left_wheel_vel = 0.0
    right_wheel_vel = 0.0
    def __init__(self):
        # call rospy.Subscriber() and rospy.Publisher() to define receiver and publisher
        # type: geometry_msgs.msg.Twist, Float64, Float64
        self.publ = rospy.Publisher('/course_agv/left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.pubr = rospy.Publisher('/course_agv/right_wheel_velocity_controller/command', Float64, queue_size=10)
        # self.subs = rospy.Subscriber('course_agv/velocity', geometry_msgs.msg.Twist, self.callback)
    # input: 
    #   data: 
    #       (data.linear.x, data.angular.z)
    # call self.kinematics(vx, vw) to compute wheel velocities
    def callback(self, data):
        # Compute self.left_wheel_vel and self.right_wheel_vel according to input
        self.kinematics(data.linear.x,data.angular.z)
        self.publish()

    # input: vx and vw of the robot
    # output: wheel velocities of two wheels
    def kinematics(self,vx,vw):
        # too simple method , TODO
        print(vx,vw)
        self.left_wheel_vel = (vx - (0.08/3+0.1)*vw)/0.08
        self.right_wheel_vel = (vx + (0.08/3+0.1)*vw)/0.08

    # Call publisher to publish wheel velocities
    def publish(self):
        self.publ.publish(self.left_wheel_vel)
        self.pubr.publish(self.right_wheel_vel)
        # print(self.left_wheel_vel,self.right_wheel_vel)

def main():
    node_name = "course_agv_kinematics"
    print("node : ",node_name)
    
    try:
        rospy.init_node(node_name)
        k = Kinematics()
        rate = rospy.Rate(rospy.get_param('~publish_rate',200))
        while not rospy.is_shutdown():
            k.publish()
            rospy.Subscriber('/course_agv/velocity', geometry_msgs.msg.Twist, k.callback)
            rospy.spin()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
