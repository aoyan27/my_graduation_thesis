#!/usr/bin/env python
#coding:utf-8
import rospy
import numpy as np
from std_msgs.msg import Float64, Int64
from sensor_msgs.msg import JointState
import math
import random

from joint_to_s.srv import *
from reward_calculation.srv import *

class test_client:
    def __init__(self):
        self.req1 = np.zeros((1, 3), dtype=np.int32)
        print self.req1
        self.req2 = 0
    def joint_to_s_client(self):
        rospy.wait_for_service('state_num')
        req_joint1 = -30
        req_joint3 = 80
        req_joint5 = 30
        try:
            joint_to_s = rospy.ServiceProxy('state_num', joint_state)
            resp = joint_to_s(req_joint1, req_joint3, req_joint5)
            #  print resp
            return resp.state
        except rospy.ServiceException, e:
            print "Service call faild: %s" % e
    def reward_calculation(self, count):
        rospy.wait_for_service('reward')
        reward_cal = rospy.ServiceProxy('reward', reward)
        resp = reward_cal(count)
        return resp.reward

    def callback(self, msg):
        print "i heard ", msg.data


def main():
    rospy.init_node('test_client', anonymous = True)
    r = rospy.Rate(10)
    #  print "aaaaa"
    step_count = 0
    test = test_client()
    rospy.Subscriber("/joint1_pose_com", Float64, test.callback)
    while not rospy.is_shutdown():
        print test.joint_to_s_client()
        #  print "bbbb"
        print test.reward_calculation(step_count)
        step_count += 1.0
        r.sleep()
        #  rospy.spin()

if __name__=="__main__":
    main()
