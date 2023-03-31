#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

class GazeboLinkPose:
    link_name = ''
    link_pose = Pose()

    def __init__(self, robot_name, link_name):
        self.robot_name = robot_name
        self.link_name = link_name
        # self.link_name_rectified = link_name.replace("::", "_")

        if not self.link_name:
            raise ValueError("'link_name' is an empty string")

        self.states_sub = rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.callback)
        self.pose_pub = rospy.Publisher(
            "/gazebo/" + (self.robot_name + '__' + self.link_name), Pose, queue_size=10)
        self.tf_pub = tf.TransformBroadcaster()

    def callback(self, data):
        try:
            ind = data.name.index(self.robot_name + "::" + self.link_name)
            self.link_pose = data.pose[ind]
        except ValueError:
            pass
    
    def publish_tf(self):
        p = self.link_pose.position
        o = self.link_pose.orientation
        self.tf_pub.sendTransform((p.x,p.y,p.z),(o.x,o.y,o.z,o.w),
                                    rospy.Time.now(),self.link_name,"map")
        self.tf_pub.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0,0,0),
                                    rospy.Time.now(),"world_base","map")

if __name__ == '__main__':
    try:
        rospy.init_node('robot_pose_tf_publisher')
        gp = GazeboLinkPose(rospy.get_param('~robot_name', 'course_agv'),
                            rospy.get_param('~link_name', 'robot_base'))
        publish_rate = rospy.get_param('~publish_rate', 100)

        rate = rospy.Rate(publish_rate)
        while not rospy.is_shutdown():
            gp.pose_pub.publish(gp.link_pose)
            gp.publish_tf()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
