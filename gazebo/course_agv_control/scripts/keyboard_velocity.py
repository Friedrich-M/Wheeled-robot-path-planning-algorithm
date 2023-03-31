#!/usr/bin/env python

import rospy
from pynput.keyboard import Key, Listener
import geometry_msgs.msg

command = {
    "vx":0.0,
    "vw":0.0,
    "step":0.1,
    "vmax":3.0
}
def limitNum(num,minNum,maxNum):
    if num > maxNum:
        return maxNum
    if num < minNum:
        return minNum
    return num
def cmd_num_func(cmd,key,value,minValue,maxValue):
    def function():
        cmd[key] = limitNum(cmd[key]+value,minValue,maxValue)
    return function
def cmd_reset_func(cmd):
    def function():
        cmd["vx"] = 0
        cmd["vw"] = 0
    return function
KEY_MAP_TABLE={
    'w' : cmd_num_func(command,"vx", command["step"],-command["vmax"],command["vmax"]),
    's' : cmd_num_func(command,"vx",-command["step"],-command["vmax"],command["vmax"]),
    'a' : cmd_num_func(command,"vw", command["step"],-command["vmax"],command["vmax"]),
    'd' : cmd_num_func(command,"vw",-command["step"],-command["vmax"],command["vmax"]),
    Key.space : cmd_reset_func(command)
}

class Publisher:
    def __init__(self):
        self.vel_pub = rospy.Publisher(
            '/course_agv/velocity',
            geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = geometry_msgs.msg.Twist()
    
    def pub(self,command):
        if rospy.is_shutdown():
            exit()
        print("publish command : vx - %.2f vw - %.2f"%(command['vx'],command['vw']))
        self.cmd.linear.x = command["vx"]
        self.cmd.angular.z = command["vw"]
        self.vel_pub.publish(self.cmd)

def press_function(map_table,publisher):
    def on_press(key):
        global NEED_EXIT
        if key == Key.esc:
            NEED_EXIT = True
            return False
        try:
            convert_key = key.char
        except AttributeError:
            convert_key = key
        # print('{0} pressed'.format(convert_key))
        if convert_key in map_table:
            map_table[convert_key]()
            publisher.pub(command)
        else:
            print('\n{0} not in table'.format(convert_key))
    return on_press

def main():
    node_name = "velocity_publisher"
    print("node : ",node_name)
    rospy.init_node(node_name,anonymous=True)
    publisher = Publisher()
    with Listener(on_press=press_function(KEY_MAP_TABLE,publisher)) as listener:
        listener.join()

if __name__ == '__main__':
    main()
