#!/usr/bin/env python
#coding:utf-8
import argparse

import rospy

import numpy as np
import chainer
import chainer.functions as F
from chainer import cuda, optimizers

from std_msgs.msg import Float64, Int64, Int32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

import math
from random import random, randint, uniform

from joint_to_s.srv import *
from reward_calculation.srv import *

import pickle
 
parser = argparse.ArgumentParser(description='q_update_client_repair')
parser.add_argument('--gpu', '-g', default=-1, type=int,
                    help='GPU ID (negative value indicates CPU)')
args = parser.parse_args()

if args.gpu >=0:
    cuda.check_cuda_available()
xp = cuda.cupy if args.gpu >=0 else np

def drange(begin, end, step):
    n = begin
    while n+step < end:
        yield n
        n += step

class agent:

    L1 = 0.064
    L2 = 0.063
    L3 = 0.250
    L4 = 0.100
    L5 = 0.100
    L6 = 0.130

    init_joint1 = 0.0
    init_joint2 = -0.0
    init_joint3 = 150.0
    init_joint5 = 45.0
    init_arm_end_x = 0.0
    init_arm_end_y = 0.0
    init_arm_end_z = 0.0

    pub_path_point = rospy.Publisher("/path_point", PointCloud, queue_size = 1)
    pub_path_point_size = rospy.Publisher("/path_point_size", Int32, queue_size = 1)
   
    wait_flag = True
    state_observation_flag = False
    state_observation_flag1 = False
    state_observation_flag2 = False
    state_observation_flag3 = False

    button_flag = False
    approach_flag = True

    def __init__(self):
        self.target_point = PointCloud()
        self.arm_end = np.zeros((3), dtype=np.float32)
        self.joint_state = np.zeros((3), dtype=np.float32)
        
        self.next_arm_end_point = PointCloud()    

    def button_point_callback(self, msg):
        self.target_point = msg
        self.button_flag = True
        #  print self.target_point

    def state_observation_flag_callback(self, msg):
	self.state_observation_flag = True
	#  print "flag !!!"

    def joint1_state_callback(self, msg):
        #  print "msg.data : ", msg.data
        self.joint_state[0] = msg.data
        #  print "joint1_state : ",self.joint_state[0]
        self.state_observation_flag1 = True

    def joint2_state_callback(self, msg):
        #  print "msg.data : ", msg.data
        self.joint_state[1] = msg.data
        #  print "joint2_state : ",self.joint_state[1]
        self.state_observation_flag2 = True

    def joint3_state_callback(self, msg):
        #  print "msg.data : ", msg.data
        self.joint_state[2] = msg.data
        #  print "joint3_state : ",self.joint_state[2]
        self.state_observation_flag3 = True

    def forward_kinematics(self, joint1_data, joint2_data, joint3_data):
        joint1_rad = joint1_data / 180.0 * math.pi
        joint2_rad = -1 * (joint2_data / 180.0 * math.pi + math.pi / 2.0)
        joint3_rad = joint3_data / 180.0 * math.pi
        joint5_rad = 45.0 / 180.0 * math.pi

        L = self.L3 * math.cos(joint2_rad) + (self.L4 + self.L5) * math.cos(joint2_rad + joint3_rad) + self.L6 * math.cos(joint2_rad + joint3_rad - joint5_rad)
        print "L : ", L

        arm_end_x = L * math.cos(joint1_rad)
        arm_end_y = L * math.sin(joint1_rad)
        arm_end_z = -(self.L1 + self.L2)  + self.L3 * math.sin(joint2_rad) + (self.L4 + self.L5) * math.sin(joint2_rad + joint3_rad) + self.L6 * math.sin(joint2_rad + joint3_rad - joint5_rad)
        return arm_end_x, arm_end_y, arm_end_z
    
    def calculation_linear(self, x, x1, x2, y1, y2):
        return (y2 - y1) / (x2 - x1) * (x - x1) + y1

    def calculation_path(self):
        target_x = self.target_point.points[0].x - 0.270
        target_y = self.target_point.points[0].y - 0.000
        target_z = self.target_point.points[0].z - 0.935
        
        path_point = PointCloud()
        path_point.header.frame_id = "/base_link"
        
        vis_path_point = PointCloud()
        vis_path_point.header.frame_id = "/base_link"
        
        for x in drange(self.init_arm_end_x, target_x, 0.001):
            y = self.calculation_linear(x, self.init_arm_end_x, target_x, self.init_arm_end_y, target_y)
            z = self.calculation_linear(x, self.init_arm_end_x, target_x, self.init_arm_end_z, target_z)
            #  print "x : ", x, ", y :  ", y, ", z : ", z
            path_point.header.stamp = rospy.Time.now()
            path_point.points.append(Point32(x, y, z))
            vis_path_point.header.stamp = rospy.Time.now()
            vis_path_point.points.append(Point32(x + 0.270, y + 0.000, z + 0.935))
        
        vis_path_point_size = len(path_point.points)
        #  print "path_point_size : ", vis_path_point_size
        self.pub_path_point.publish(vis_path_point) 
        self.pub_path_point_size.publish(vis_path_point_size)
        
        return path_point

    def function1(self, arm_end_x, arm_end_y, arm_end_z, joint2_rad, joint3_rad):
        joint5_rad = 45.0 /180.0 * math.pi
        L = math.sqrt(arm_end_x**2 + arm_end_y**2)
        answer = L - (self.L3 * math.cos(joint2_rad) + (self.L4 + self.L5) * math.cos(joint2_rad + joint3_rad) + self.L6 * math.cos(joint2_rad + joint3_rad - joint5_rad))
        return answer

    def function2(self, arm_end_x, arm_end_y, arm_end_z, joint2_rad, joint3_rad):
        joint5_rad = 45.0 /180.0 * math.pi
        answer = arm_end_z - (-(self.L1 + self.L2) + self.L3 * math.sin(joint2_rad) + (self.L4 + self.L5) * math.sin(joint2_rad + joint3_rad) + self.L6 * math.sin(joint2_rad + joint3_rad - joint5_rad))
        return answer
    
    def J11(self, joint2_rad, joint3_rad):
        joint5_rad = 45.0 /180.0 * math.pi
        differential = -1 * (-1 * self.L3 * math.sin(joint2_rad) - (self.L4 + self.L5) * math.sin(joint2_rad + joint3_rad) - self.L6 * math.sin(joint2_rad + joint3_rad - joint5_rad))
        return differential

    def J12(self, joint2_rad, joint3_rad):
        joint5_rad = 45.0 /180.0 * math.pi
        differential = -1 * (-1 * (self.L4 + self.L5) * math.sin(joint2_rad + joint3_rad) - self.L6 * math.sin(joint2_rad + joint3_rad - joint5_rad))
        return differential

    def J21(self, joint2_rad, joint3_rad):
        joint5_rad = 45.0 /180.0 * math.pi
        differential = -1 * (self.L3 * math.cos(joint2_rad) + (self.L4 + self.L5) * math.cos(joint2_rad + joint3_rad) + self.L6 * math.cos(joint2_rad + joint3_rad - joint5_rad))
        return differential

    def J22(self, joint2_rad, joint3_rad):
        joint5_rad = 45.0 /180.0 * math.pi
        differential = -1 * ((self.L4 + self.L5) * math.cos(joint2_rad + joint3_rad) + self.L6 * math.cos(joint2_rad + joint3_rad - joint5_rad))
        return differential
    
    def ucal(self, arm_end_x, arm_end_y, arm_end_z, joint2_rad, joint3_rad):
        det = self.J11(joint2_rad, joint3_rad) * self.J22(joint2_rad, joint3_rad) - self.J12(joint2_rad, joint3_rad) * self.J21(joint2_rad, joint3_rad)

        u_joint2 = -1 / det * (self.J22(joint2_rad, joint3_rad) * self.function1(arm_end_x, arm_end_y, arm_end_z, joint2_rad, joint3_rad) - self.J12(joint2_rad, joint3_rad) * self.function2(arm_end_x, arm_end_y, arm_end_z, joint2_rad, joint3_rad))

        u_joint3 = 1 / det * (self.J21(joint2_rad, joint3_rad) * self.function1(arm_end_x, arm_end_y, arm_end_z, joint2_rad, joint3_rad) - self.J11(joint2_rad, joint3_rad) * self.function2(arm_end_x, arm_end_y, arm_end_z, joint2_rad, joint3_rad))

        return u_joint2, u_joint3
       
    def newton2(self, arm_end_x, arm_end_y, arm_end_z, joint2_rad, joint3_rad):
        eps = 1e-10
        max_repeat = 100000
        joint2 = joint2_rad
        joint3 = joint3_rad
        for i in range(max_repeat):
            joint2_old = joint2
            joint3_old = joint3
            u_joint2, u_joint3 = self.ucal(arm_end_x, arm_end_y, arm_end_z, joint2, joint3)
            joint2 += u_joint2
            joint3 += u_joint3

            error1 = math.fabs(joint2_old - joint2)
            error2 = math.fabs(joint3_old - joint3)

            if error1 <= eps and error2 <= eps:
                print "epoch : ", i
                break

        return joint2, joint3

    def inverse_kinematics(self, arm_end_x, arm_end_y, arm_end_z):
        j2_rad = -1 * (-50.0 /180.0 * math.pi + math.pi / 2.0)
        j3_rad = 105.0 / 180.0 * math.pi
        joint1 = -1 * math.atan2(arm_end_y, arm_end_x)
        joint2, joint3 = self.newton2(arm_end_x, arm_end_y, arm_end_z, j2_rad, j3_rad)

        joint1 = joint1 / math.pi * 180
        joint2 = (-1 * joint2 - math.pi / 2.0) / math.pi * 180.0
        joint3 = joint3 / math.pi * 180

        print "joint1 : ", joint1
        print "joint2 : ", joint2
        print "joint3 : ", joint3

        return joint1, joint2, joint3
        

    def main(self):
        rospy.init_node('q_update_old_method')

        rospy.Subscriber("/button_point", PointCloud, self.button_point_callback)

        rospy.Subscriber("/state_observation_flag", Int64, self.state_observation_flag_callback)
        rospy.Subscriber("/joint1_state", Float64, self.joint1_state_callback);
        rospy.Subscriber("/joint2_state", Float64, self.joint2_state_callback);
        rospy.Subscriber("/joint3_state", Float64, self.joint3_state_callback);

        pub = rospy.Publisher("/vis_button_point", PointCloud, queue_size = 1)
        pub_1 = rospy.Publisher("/joint1_pose_com", Float64, queue_size = 1)
        pub_2 = rospy.Publisher("/joint2_pose_com", Float64, queue_size = 1)
        pub_3 = rospy.Publisher("/joint3_pose_com", Float64, queue_size = 1)
        pub_5 = rospy.Publisher("/joint5_pose_com", Float64, queue_size = 1)


        loop_rate = rospy.Rate(100)

        self.init_arm_end_x, self.init_arm_end_y, self.init_arm_end_z = self.forward_kinematics(self.init_joint1, self.init_joint2, self.init_joint3)
        
        count = 0
        i = 0
        x_addition = 0.0
        j1 = self.init_joint1
        j2 = self.init_joint2
        j3 = self.init_joint3
        j5 = self.init_joint5

        while not rospy.is_shutdown():
            pub.publish(self.target_point)
            if self.wait_flag:
                print "wait 1 second!!", count
                count += 1
                #  if count == 1:
                    #  pub_1.publish(j1)
                    #  pub_2.publish(j2)
                    #  pub_3.publish(j3)
                    #  pub_5.publish(j5)
                if count == 50:
                    pub_1.publish(self.init_joint1)
                    pub_2.publish(self.init_joint2)
                    pub_3.publish(self.init_joint3)
                    pub_5.publish(self.init_joint5)
                if count == 100:
                    if self.button_flag:
                        target_x = self.target_point.points[0].x - 0.270
                        target_y = self.target_point.points[0].y - 0.000
                        target_z = self.target_point.points[0].z - 0.935
                        j1_temp, j2_temp, j3_temp = self.inverse_kinematics(target_x, target_y, target_z)
                        print "target_x : ", target_x
                        print "target_y : ", target_y
                        print "target_z : ", target_z
                        print "joint1 : ", j1_temp
                        print "joint2 : ", j2_temp
                        print "joint3 : ", j3_temp
                        self.next_arm_end_point = self.calculation_path()
                        #  self.next_arm_end_point = self.target_point
                        self.wait_flag = False
                        self.state_observation_flag = True
                        self.state_observation_flag1 = True
                        self.state_observation_flag2 = True
                        self.state_observation_flag3 = True
                        self.button_flag = False
                    else:
                        print "one_more!!!"
                    count = 0
            else:
                if self.state_observation_flag and self.state_observation_flag1 and self.state_observation_flag2 and self.state_observation_flag3:
                    if self.approach_flag:
                        if i < len(self.next_arm_end_point.points):
                            next_arm_end_x = self.next_arm_end_point.points[i].x
                            next_arm_end_y = self.next_arm_end_point.points[i].y
                            next_arm_end_z = self.next_arm_end_point.points[i].z

                            print "next_arm_end_x : ", next_arm_end_x
                            print "next_arm_end_y : ", next_arm_end_y
                            print "next_arm_end_z : ", next_arm_end_z
                            
                            j1, j2, j3 = self.inverse_kinematics(next_arm_end_x, next_arm_end_y, next_arm_end_z)
                            j5 = 45.0
                            print "j1 : ", j1
                            print "j2 : ", j2 
                            print "j3 : ", j3
                            print "j5 : ", j5

                            j2_rad = -1 * (j2 / 180.0 * math.pi + math.pi / 2.0)
                            j3_rad = j3 / 180.0 * math.pi
                            
                            ans1 = self.function1(next_arm_end_x, next_arm_end_y, next_arm_end_z, j2_rad, j3_rad)

                            ans2 = self.function2(next_arm_end_x, next_arm_end_y, next_arm_end_z, j2_rad, j3_rad)
                            print "ans1 : ", ans1
                            print "ans2 : ", ans2

                            pub_1.publish(j1)
                            pub_2.publish(j2)
                            pub_3.publish(j3)
                            pub_5.publish(j5)

                            i += 1

                            self.observation_flag = False
                            self.observation_flag1 = False
                            self.observation_flag2 = False
                            self.observation_flag3 = False
                        else:
                            self.approach_flag = False
                            i = 0
                    else:
                        print "now pushing!!!!!!!!!"
                        joint1_rad = j1 / 180.0 * math.pi
                        joint2_rad = -1 * (j2 / 180.0 * math.pi + math.pi / 2.0)
                        joint3_rad = j3 / 180.0 * math.pi
                        joint5_rad = j5 / 180.0 * math.pi

                        arm_end_x, arm_end_y, arm_end_z = self.forward_kinematics(j1, j2, j3)
                        
                        print "end position x : ", arm_end_x + 0.270
                        print "end position y : ", arm_end_y + 0.000
                        print "end position z : ", arm_end_z + 0.935
                        
                        print ""

                        next_arm_end_x = arm_end_x + x_addition
                        next_arm_end_y = arm_end_y
                        next_arm_end_z = arm_end_z

                        print "next end position x : ", next_arm_end_x + 0.270
                        print "next end position y : ", next_arm_end_y + 0.000
                        print "next end position z : ", next_arm_end_z + 0.935
                        
                        print ""


                        I  = math.sqrt(next_arm_end_x**2 + next_arm_end_y**2)
                        #  print "I : ", I
                        I_L3off = self.L3 * math.cos(joint2_rad)
                        Z_L3off = self.L3 * math.sin(joint2_rad)
                        ld = math.sqrt((I - I_L3off)**2 + (next_arm_end_z + (self.L1 + self.L2) - Z_L3off)**2)
                        #  print "ld : ", ld

                        joint5_com_limit = ((self.L4 + self.L5)**2 + self.L6**2 - ld**2) / (2 * (self.L4 + self.L5) * self.L6)
                        joint3_com_limit = ((self.L4 + self.L5)**2 + ld**2 - self.L6**2) / (2 * (self.L4 + self.L5) * ld)
                        
                        if (joint5_com_limit <= 1 and joint3_com_limit <= 1) and x_addition < 0.020:
                            joint1_com = math.atan2(next_arm_end_y, next_arm_end_x)
                            joint5_com = math.pi - math.acos(((self.L4 + self.L5)**2 + self.L6**2 - ld**2) / (2 * (self.L4 + self.L5) * self.L6))
    
                            joint3_com = math.acos(((self.L4 + self.L5)**2 + ld**2 - self.L6**2) / (2 * (self.L4 + self.L5) * ld)) + math.atan2((next_arm_end_z + (self.L1 + self.L2) - Z_L3off), (I - I_L3off)) - joint2_rad
                            
                            
                            joint1_com = joint1_com / math.pi * 180.0
                            joint2_com = j2
                            joint3_com = joint3_com / math.pi * 180.0
                            joint5_com = joint5_com / math.pi * 180.0
                            print "joint1 command : ", joint1_com
                            print "joint2 command : ", joint2_com
                            print "joint3 command : ", joint3_com
                            print "joint5 command : ", joint5_com
                            
                            print ""
    
                            x_addition += 0.001
                            print "x add : ", x_addition
                        else:
                            x_addition = 0.0
                            print "can not reach next end position!!!!"
                            print "this episode finish!!!!!!!!!!!!!!"
                            self.wait_flag = True
                            self.approach_flag = True
                            
                        self.state_observation_flag = False
                        self.state_observation_flag1 = False
                        self.state_observation_flag2 = False
                        self.state_observation_flag3 = False

                        pub_1.publish(joint1_com)
                        print "joint1 command publish!!"
                        pub_2.publish(joint2_com)
                        print "joint2 command publish!!"
                        pub_3.publish(joint3_com)
                        print "joint3 command publish!!"
                        pub_5.publish(joint5_com)
                        print "joint5 command publish!!"
            
            loop_rate.sleep()


    
if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
