#!/usr/bin/env python
#coding:utf-8
import argparse

import rospy

import numpy as np
import chainer
import chainer.functions as F
from chainer import cuda, optimizers

from std_msgs.msg import Float64, Int64
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

import math
from random import random, randint, uniform

from joint_to_s.srv import *
from reward_calculation.srv import *

import pickle


if __name__=="__main__":
    rospy.init_node('sample_botton_point')

    pub = rospy.Publisher("button_point", PointCloud, queue_size = 1)

    loop_rate = rospy.Rate(10)

    pi = 3.141592
    
    L1 = 0.064
    L2 = 0.063
    L3 = 0.250
    L4 = 0.100
    L5 = 0.100
    L6 = 0.130
    
    joint1_rad = 0.0 / 180.0 * pi
    #  joint2_rad = -(1.515/self.pi*180.0 - 90.0) / 180.0 * self.pi
    joint2_rad = -1 * (-1.515 + pi / 2.0)
    joint3_rad = 55.0 / 180.0 * pi
    joint5_rad = 45.0 / 180.0 * pi
    L = L3 * math.cos(joint2_rad) + (L4 + L5) * math.cos(joint2_rad + joint3_rad) + L6 * math.cos(joint2_rad + joint3_rad - joint5_rad) - 0.010

    target_init_y = 0.15
    target_init_x = math.sqrt(L**2 - target_init_y**2) + 0.270
    target_init_z = 0.90
    target = PointCloud()
    target.header.frame_id = "/base_link"
    
    target.points.append(Point32(target_init_x, target_init_y, target_init_z))

    i = 0.0
    while not rospy.is_shutdown():
        target_point_y = 0.20 + i
        target.points[0].y = target_point_y
        target.points[0].x = math.sqrt(L**2 - target_point_y**2) + 0.270
        #  target.points[0].y = -0.034
        #  target.points[0].x = 0.803
        target.points[0].z = 0.90
        target.header.stamp = rospy.Time.now() 
        pub.publish(target)
        print "publish ", target
        #  i += 0.01
        loop_rate.sleep()
