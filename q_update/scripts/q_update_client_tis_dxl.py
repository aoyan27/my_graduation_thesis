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
from button_converter.srv import *

from dynamixel_controllers.srv import *

import pickle
 
parser = argparse.ArgumentParser(description='q_update_client_repair')
parser.add_argument('--gpu', '-g', default=0, type=int,
                    help='GPU ID (negative value indicates CPU)')
args = parser.parse_args()

if args.gpu >=0:
    cuda.check_cuda_available()
xp = cuda.cupy if args.gpu >=0 else np

class agent:
	pi = 3.141592

        init_state_joint1 = -175
	init_state_joint3 = 150
	init_state_joint5 = -95

        init_joint1 = 0
        init_joint3 = 65
        init_joint5 = 45

        init_next_joint1 = init_joint1
        init_next_joint3 = init_joint3
        init_next_joint5 = init_joint5
	
	num_a = 3**3

	ALPHA = 0.5
	GAMMA = 0.9
        EPSILON = 0.0

	#  wait_flag = False
	wait_flag = True
	select_action_flag = False
	q_update_flag = False
	state_observation_flag = False
	state_observation_flag1 = False
	state_observation_flag3 = False
	state_observation_flag5 = False

        #  mode_dqn_flag = True
        mode_dqn_flag = False

        botton_wait_flag = True

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
        
        def __init__(self):
		self.joint_state = np.zeros((3), dtype=np.float32)
		#  print self.joint_state
		self.action_num = 0
		self.reward = 0.0

                self.target_point = PointCloud()
                self.target_init_x = self.L + 0.270
                self.target_init_y = 0.000
                self.target_init_z = 0.960
                
                self.arm_end = np.zeros((3), dtype=np.float32)
                self.arm_end_com = np.zeros((3), dtype=np.float32)

		self.num_step = 0
                self.num_episode = 0

                f = open('/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_test16_angeal/dqn_arm_model_30000.dat', 'rb')
                self.model = pickle.load(f)
                if args.gpu >= 0:
                    self.model.to_gpu()
                self.optimizer = optimizers.SGD()
                self.optimizer.setup(self.model)
                self.q_list = chainer.Variable(xp.zeros((1, 27), dtype=xp.float32))

                self.action = 0
                self.joint1 = self.init_state_joint1
                self.joint3 = self.init_state_joint3
                self.joint5 = self.init_state_joint5
                self.next_joint1 = self.init_state_joint1
                self.next_joint3 = self.init_state_joint3
                self.next_joint5 = self.init_state_joint5
                
                self.joint1_com = self.init_state_joint1
                self.joint3_com = self.init_state_joint3
                self.joint5_com = self.init_state_joint5

	def state_observation_flag_callback(self, msg):
	    self.state_observation_flag = True
	    #  print "flag !!!"
        
        def joint1_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[0] = msg.data
            print "joint1_state : ",self.joint_state[0]
	    self.state_observation_flag1 = True
 
        def joint3_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[1] = msg.data
            print "joint3_state : ",self.joint_state[1]
	    self.state_observation_flag3 = True

        def joint5_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[2] = msg.data
            print "joint5_state : ",self.joint_state[2]
	    self.state_observation_flag5 = True        

        def reward_calculation_client(self, req_reward):
            #  rospy.wait_for_service('reward')
            try:
                reward_calculation = rospy.ServiceProxy('reward', reward)
                resp = reward_calculation(req_reward)
                #  print "reward : ", resp.reward
                return resp.reward
            except rospy.ServiceException, e:
                print "Service call faild : %s" % e
        
        def button_point_client(self, req_target_point):
            #  rospy.wait_for_service('reward')
            try:
                button_converter = rospy.ServiceProxy('button_point2', button_point)
                resp = button_converter(req_target_point)
                return resp.point_x, resp.point_y, resp.point_z
            except rospy.ServiceException, e:
                print "Service call faild : %s" % e
        
        def set_speed_client1(self, req_speed):
            #  rospy.wait_for_service('reward')
            try:
                set_speed = rospy.ServiceProxy('/servo4_controller/set_speed', SetSpeed)
                resp = set_speed(req_speed)
                #  print "reward : ", resp.reward
            except rospy.ServiceException, e:
                print "Service call faild : %s" % e
        
        def set_speed_client2(self, req_speed):
            #  rospy.wait_for_service('reward')
            try:
                set_speed = rospy.ServiceProxy('/servo6_controller/set_speed', SetSpeed)
                resp = set_speed(req_speed)
                #  print "reward : ", resp.reward
            except rospy.ServiceException, e:
                print "Service call faild : %s" % e
        
        def mean(self, data1, data2):
            return (data1 + data2) / 2.0
        
        def cal_target_x(self, y_data):
            return math.sqrt(self.L**2 - y_data**2) + 0.270

        def forward(self, joint1_data, joint3_data, joint5_data):
            joint1_data_float = float(joint1_data -self.mean(50.0, -50.0) ) / (50.0 - (-50.0))
            joint3_data_float = float(joint3_data - self.mean(90.0, -10.0)) / (90.0 - (-10.0))
            joint5_data_float = float(joint5_data -self.mean(90.0, -10.0) ) / (90.0 - (-10.0))

            target_data_x = float((self.target_point.points[0].x - self.mean(self.cal_target_x(0.000), self.cal_target_x(0.160))) / (self.cal_target_x(0.000) - self.cal_target_x(0.160)))
            target_data_y = float((self.target_point.points[0].y - self.mean(0.160, -0.160)) / (0.160 - (-0.160)))
            target_data_z = float((self.target_point.points[0].z - self.mean(1.000, 0.900)) / (1.000 - 0.900))

            x = chainer.Variable(xp.array([[joint1_data_float, joint3_data_float, joint5_data_float, target_data_x, target_data_y, target_data_z]], dtype=xp.float32))
            print "x :  ", x.data
            #  x.data = x.data / xp.linalg.norm(x.data)
            #  print "x(nomal) : ", x.data
            y = None

            h1 = F.relu(self.model.l1(x))
            #  print "norm h1(before) : ", self.my_norm(h1.data)
            #  h1.data = h1.data / self.my_norm(h1.data)
            #  print "norm h1(after) : ", self.my_norm(h1.data)
            #  h1.data = h1.data /np.linalg.norm(h1.data)
            #  print "h1 : ", h1.data
            
            h2 = F.relu(self.model.l2(h1))
            #  print "norm h2(before) : ", self.my_norm(h2.data)
            #  h2.data = h2.data / self.my_norm(h2.data)
            #  print "norm h2(after) : ", self.my_norm(h2.data)
            #  h2.data = h2.data / np.linalg.norm(h2.data)
            #  print "h2 : ", h2.data
            
            h3 = F.relu(self.model.l3(h2))
            #  print "norm h3(before) : ", self.my_norm(h3.data)
            #  h3.data = h3.data / self.my_norm(h3.data)
            #  print "norm h3(after) : ", self.my_norm(h3.data)
            #  h3.data = h3.data / np.linalg.norm(h3.data)
            #  print "h3 : ", h3.data
            
            h4 = F.relu(self.model.l4(h3))
            #  print "norm h4(before) : ", self.my_norm(h4.data)
            #  h4.data = h4.data / self.my_norm(h4.data)
            #  print "norm h4(after) : ", self.my_norm(h4.data)
            #  h4.data = h4.data / np.linalg.norm(h4.data)
            #  print "h4 : ", h4.data
            
            h5 = F.relu(self.model.l5(h4)) 
            #  print "norm h5(before) : ", self.my_norm(h5.data)
            #  h5.data = h5.data / self.my_norm(h5.data)
            #  print "norm h5(after) : ", self.my_norm(h5.data)
            #  h5.data = h5.data / np.linalg.norm(h5.data)
            #  print "h5 : ", h5.data
            h6 = F.relu(self.model.l6(h5))
            y = self.model.l7(h6)
            print "y : ", y.data

            return y
            	
	def select_action(self, joint1, joint3, joint5):
            self.q_list = self.forward(joint1, joint3, joint5)
            i_max = []
	    i_max.append(0)
	    max_temp = self.q_list.data[0][0]
	    for i in range(1, self.num_a):
                if self.q_list.data[0][i]>max_temp:
                    max_temp = self.q_list.data[0][i]
                    i_max = []
		    i_max.append(i)
		elif self.q_list.data[0][i] == max_temp:
		    i_max.append(i)
	    a = i_max[randint(0, len(i_max)-1)]
	    return a

	def epsilon_greedy(self, joint1, joint3, joint5):
	    if self.EPSILON > random():
		a = randint(0, self.num_a-1)
                print "random select!!"
	    else:
		a = self.select_action(joint1, joint3, joint5)
                print "argmax Q select!!"
	    return a
        
        def main(self):
            rospy.init_node('q_update_client_dqn')
            
            rospy.Subscriber("/state_observation_flag", Int64, self.state_observation_flag_callback)
            rospy.Subscriber("/joint1_state", Float64, self.joint1_state_callback)
            rospy.Subscriber("/joint3_state", Float64, self.joint3_state_callback)
            rospy.Subscriber("/joint5_state", Float64, self.joint5_state_callback)

            
            pub_1 = rospy.Publisher("/joint1_pose", Float64, queue_size = 1)
            pub_3 = rospy.Publisher("/joint3_pose", Float64, queue_size = 1)
            pub_5 = rospy.Publisher("/joint5_pose", Float64, queue_size = 1)
            pub_6 = rospy.Publisher("/action_num", Int64, queue_size = 1)
            pub_7 = rospy.Publisher("/num_step", Int64, queue_size = 1)
            pub_8 = rospy.Publisher("/num_episode", Int64, queue_size = 1)

            pub_9 = rospy.Publisher("/target_point", PointCloud, queue_size = 1)
            pub_10 = rospy.Publisher("/vis_target_point", PointCloud, queue_size = 1)

            pub_11 = rospy.Publisher("/joint1_pose_com", Float64, queue_size = 1)
            pub_13 = rospy.Publisher("/joint3_pose_com", Float64, queue_size = 1)
            pub_15 = rospy.Publisher("/joint5_pose_com", Float64, queue_size = 1)
            
            loop_rate = rospy.Rate(100)
            
            filename_result = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_result.txt"

            count = 0
            count_temp = 0

            self.joint1 = self.init_joint1
            self.joint3 = self.init_joint3
            self.joint5 = self.init_joint5
            print "joint1 : ", self.joint1
            print "joint3 : ", self.joint3
            print "joint5 : ", self.joint5

            self.next_joint1 = self.init_joint1
            self.next_joint3 = self.init_joint3
            self.next_joint5 = self.init_joint5
            self.joint_state[0] = self.init_joint1
            self.joint_state[1] = self.init_joint3
            self.joint_state[2] = self.init_joint5
            print "next joint1 : ", self.next_joint1
            print "next joint3 : ", self.next_joint3
            print "next joint5 : ", self.next_joint5

            step_count = 0
            episode_count = 0
            
            episode_now = 0
            episode_past = 0

            temp_count = 0

            loss_list = []

            target_vis = PointCloud()
            target_vis.header.frame_id = "/base_link"
            target_vis.header.stamp = rospy.Time.now() 

            self.target_point.header.frame_id = "/base_link"
            self.target_point.header.stamp = rospy.Time.now()
            self.target_point.points.append(Point32(self.target_init_x, self.target_init_y, self.target_init_z))
            print type(self.target_point.points[0].x)
            print "target_point : ", self.target_point

            print "Q Learning Start!!"

            x_addition = 0
            
            #  print "L : ", self.L
            #  print "x : ", self.L * math.cos(0.0)+0.270
            #  rand_target_vis_y = -0.08
            
            pub_11_flag = False
            pub_13_flag = False

            while not rospy.is_shutdown():
                if episode_count == 0:
                    if step_count == 0:    
                        pub_9.publish(self.target_point) 
                        self.set_speed_client1(0.04)
                        self.set_speed_client2(0.08)
                        
                #  rand_target_vis_x = math.sqrt(self.L**2 - rand_target_vis_y**2) + 0.270
                #  rand_target_vis_z = self.target_init_z
                
                #  if rand_target_vis_y <=0.08:    
                    #  target_vis.points.append(Point32(rand_target_vis_x, rand_target_vis_y, rand_target_vis_z))
                    #  pub_10.publish(target_vis)
                    #  rand_target_vis_y += 0.01 
                
                if self.wait_flag:
                    #  print "wait 10 seconds!!"
                    count += 1
                    if count == 150:
                        self.wait_flag = False
                        self.select_action_flag = False
                        self.q_update_flag = False
                        self.mode_dqn_flag = True
                        self.botton_wait_flag = True
                        if episode_count == 0:
                            self.state_observation_flag = True
                            self.state_observation_flag1 = True
                            self.state_observation_flag3 = True
                            self.state_observation_flag5 = True
                        
                        pub_11_flag = False
                        pub_13_flag = False
                        
                        count = 0
                    if count == 10:
                        self.action_num = 0
                        self.joint5 = self.init_next_joint5
                        self.next_joint5 = self.init_next_joint5
                        self.reward = 0.0
                        pub_1.publish(self.joint1)
                        pub_3.publish(self.joint3)
                        pub_5.publish(self.joint5)
                        pub_6.publish(self.action_num)
                        print "publish joint5 : ", self.joint5
                    if count == 50:
                        self.joint3 = self.init_next_joint3
                        self.next_joint3 = self.init_next_joint3
                        pub_1.publish(self.joint1)
                        pub_3.publish(self.joint3)
                        pub_5.publish(self.joint5)
                        pub_6.publish(self.action_num)
                        print "publish joint3 : ", self.joint3
                    if count == 100:
                        self.joint1 = self.init_next_joint1
                        self.next_joint1 = self.init_next_joint1
                        pub_1.publish(self.joint1)
                        pub_3.publish(self.joint3)
                        pub_5.publish(self.joint5)
                        pub_6.publish(self.action_num)
                        print "publish joint1 : ", self.joint1
                    pub_1.publish(self.joint1)
                    pub_3.publish(self.joint3)
                    pub_5.publish(self.joint5)
                else:
                    if self.botton_wait_flag:
                        print "botton waiting!!!!!!"
                        #  rospy.sleep(1)
                        self.target_point.header.stamp = rospy.Time.now()
                        self.target_point.points[0].x, self.target_point.points[0].y, self.target_point.points[0].z, = self.button_point_client(1)
                        self.target_point.points[0].x = math.sqrt(self.L**2 - self.target_point.points[0].y**2) + 0.270
                        self.target_point.points[0].z = 0.96
                        print "traget : ",self.target_point
                        self.botton_wait_flag = False
                    else:
                        if self.mode_dqn_flag:
                            if self.select_action_flag:
                                self.action = self.epsilon_greedy(self.joint1, self.joint3, self.joint5)
                                #  self.action = 8
                                self.action_num = self.action
                                print "self.action_num : ", self.action_num
                                pub_1.publish(self.joint1) 
                                pub_3.publish(self.joint3) 
                                pub_5.publish(self.joint5) 
                                pub_6.publish(self.action_num)
                                self.select_action_flag = False

                            if self.state_observation_flag and self.state_observation_flag1 and self.state_observation_flag3 and self.state_observation_flag5:
                                print "self.joint_state[0] : ",self.joint_state[0]
                                print "self.joint_state[1] : ",self.joint_state[1]
                                print "self.joint_state[2] : ",self.joint_state[2]
                                
                                print "now joint1 : ", self.joint1
                                print "now joint3 : ", self.joint3
                                print "now joint5 : ", self.joint5
                                
                                self.next_joint1 = self.joint_state[0]
                                self.next_joint3 = self.joint_state[1]
                                self.next_joint5 = self.joint_state[2]
                                print "next joint1 : ", self.next_joint1
                                print "next joint3 : ", self.next_joint3
                                print "next joint5 : ", self.next_joint5

                                self.reward = self.reward_calculation_client(step_count)
                                print "reward : ", self.reward
                                #  self.select_action_flag = True
                                self.q_update_flag = True
                                self.state_observation_flag = False
                                self.state_observation_flag1 = False
                                self.state_observation_flag3 = False
                                self.state_observation_flag5 = False

                            if self.q_update_flag:
                                self.select_action_flag = True
                                self.q_update_flag = False
                                step_count += 1
                                print "episode : %d " % episode_count,
                                print "step : %d " % step_count,
                                print "now joint1 : %d " % self.joint1,
                                print "now joint3 : %d " % self.joint3,
                                print "now joint5 : %d " % self.joint5,
                                print "now action : %d" % self.action,
                                #  print "loss : ", np.max(loss.data),
                                print "reward : %.1f  " % self.reward,
                                print "EPSILON : %.5f " % self.EPSILON
                                print ""

                            #  print ""

                            if self.reward >= 1:
                                print "episode : %d " % episode_count,
                                print "step : %d " % step_count,
                                print "now joint1 : %d " % self.joint1,
                                print "now joint3 : %d " % self.joint3,
                                print "now joint5 : %d " % self.joint5,
                                print "now action : %d" % self.action,
                                #  print "loss average : %.3f " % (sum(loss_list)/len(loss_list)),
                                print "reward : %.1f  " % self.reward,
                                print "EPSILON : %.5f " % self.EPSILON,
                                print "succsess!!"
                                print ""

                                temp_result = np.array(([[episode_count, step_count]]), dtype=np.int32)
                                if episode_count == 0:
                                    test_result = temp_result
                                else:
                                    test_result = np.r_[test_result, temp_result]

                                
                                #  rand_target_y = uniform(self.target_init_y-0.08, self.target_init_y+0.08)
                                rand_target_y = self.target_init_y
                                #  rand_target_x = self.target_init_x
                                rand_target_x = math.sqrt(self.L**2 - rand_target_y**2) + 0.270 
                                rand_target_z = self.target_init_z
                                self.target_point.header.stamp = rospy.Time.now()
                                self.target_point.points[0].x = rand_target_x
                                self.target_point.points[0].y = rand_target_y
                                self.target_point.points[0].z = rand_target_z
                                
                                step_count = 0
                                episode_count += 1
                                episode_now = episode_count

                                #  self.action_num = 0
                                #  self.joint1 = self.init_next_joint1
                                #  self.joint3 = self.init_next_joint3
                                #  self.joint5 = self.init_next_joint5
                                #  pub_1.publish(self.joint1)
                                #  pub_3.publish(self.joint3)
                                #  pub_5.publish(self.joint5)
                                #  pub_6.publish(self.action_num)
                                #  loss_list = []

                                #  pub_9.publish(self.target_point)
                                
                                self.mode_dqn_flag = False
                                self.state_observation_flag = True
                                self.state_observation_flag1 = True
                                self.state_observation_flag3 = True
                                self.state_observation_flag5 = True
                                #  self.wait_flag = True
                            else:
                                if step_count < 18:

                                    self.joint1 = self.next_joint1
                                    self.joint3 = self.next_joint3
                                    self.joint5 = self.next_joint5

                                    episode_past = episode_now
                                else:
                                    print "episode : %d " % episode_count,
                                    print "step : %d " % step_count,
                                    print "now joint1 : %d " % self.joint1,
                                    print "now joint3 : %d " % self.joint3,
                                    print "now joint5 : %d " % self.joint5,
                                    print "now action : %d" % self.action,
                                    #  print "loss average : %.3f " % (sum(loss_list)/len(loss_list)),
                                    print "reward : %.1f  " % self.reward,
                                    print "EPSILON : %.5f " % self.EPSILON,
                                    print "failuer!!"
                                    print ""

                                    temp_result = np.array(([[episode_count, step_count]]), dtype=np.int32)
                                    if episode_count == 0:
                                        test_result = temp_result
                                    else:
                                        test_result = np.r_[test_result, temp_result]
                                    
                                    #  rand_target_y = uniform(self.target_init_y-0.08, self.target_init_y+0.08)
                                    rand_target_y = self.target_init_y
                                    #  rand_target_x = self.target_init_x
                                    rand_target_x = math.sqrt(self.L**2 - rand_target_y**2) + 0.270 
                                    rand_target_z = self.target_init_z
                                    self.target_point.header.stamp = rospy.Time.now()
                                    self.target_point.points[0].x = rand_target_x
                                    self.target_point.points[0].y = rand_target_y
                                    self.target_point.points[0].z = rand_target_z
                                
                                    step_count = 0
                                    episode_count += 1
                                    episode_now = episode_count

                                    self.action_num = 0
                                    #  self.joint1 = self.init_next_joint1
                                    #  self.joint3 = self.init_next_joint3
                                    #  self.joint5 = self.init_next_joint5
                                    #  pub_1.publish(self.joint1)
                                    #  pub_3.publish(self.joint3)
                                    #  pub_5.publish(self.joint5)
                                    #  pub_6.publish(self.action_num)
                                    #  loss_list = []

                                    #  pub_9.publish(self.target_point)
                                    
                                    self.mode_dqn_flag = False
                                    self.state_observation_flag = True
                                    self.state_observation_flag1 = True
                                    self.state_observation_flag3 = True
                                    self.state_observation_flag5 = True
                                    #  self.wait_flag = True

                            #  if math.fabs(episode_now - episode_past) > 1e-6:
                                #  if self.EPSILON > 0.1000:
                                    #  self.EPSILON -= 0.0001

                            self.num_step = step_count
                            pub_7.publish(self.num_step)
                            self.num_episode = episode_count
                            pub_8.publish(self.num_episode)
                            
                            #  if episode_count%50 == 0:
                                #  model_filename = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model_%d.dat" % episode_count
                                #  f = open(model_filename, 'w')
                                #  pickle.dump(self.model, f)

                            #  if episode_count > 10000:
                                #  np.savetxt(filename_result, test_result, fmt="%d", delimiter=",")
                                #  f = open('/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model.dat', 'w')
                                #  pickle.dump(self.model, f)
                                #  print "Finish!!!"
                                #  break
                        else:
                            if self.state_observation_flag and self.state_observation_flag1 and self.state_observation_flag3 and self.state_observation_flag5:
                                pub_11_flag = False
                                pub_13_flag = False

                                self.joint1 = self.next_joint1
                                self.joint3 = self.next_joint3
                                self.joint5 = self.next_joint5
                                print "state_joint1 : ", self.joint1
                                print "state_joint3 : ", self.joint3
                                print "state_joint5 : ", self.joint5
        
                                print ""
        
                                joint1_rad = self.joint1 / 180.0 * self.pi
                                #  joint2_rad = 5.0 / 180.0 * self.pi
                                joint2_rad = -1 * (-1.515 + self.pi / 2.0)
                                joint3_rad = self.joint3 / 180.0 * self.pi
                                joint5_rad = self.joint5 / 180.0 * self.pi
        
                               
                                L = self.L3 * math.cos(joint2_rad) + (self.L4 + self.L5) * math.cos(joint2_rad + joint3_rad) + self.L6 * math.cos(joint2_rad + joint3_rad - joint5_rad)
                                #  print "L : ",L
        
                                self.arm_end[0] = L * math.cos(joint1_rad)
                                self.arm_end[1] = L * math.sin(joint1_rad)
                                self.arm_end[2] = -(self.L1 + self.L2)  + self.L3 * math.sin(joint2_rad) + (self.L4 + self.L5) * math.sin(joint2_rad + joint3_rad) + self.L6 * math.sin(joint2_rad + joint3_rad - joint5_rad)
        
                                print "end position x : ", self.arm_end[0] + 0.270
                                print "end position y : ", self.arm_end[1] + 0.000
                                print "end position z : ", self.arm_end[2] + 0.935
                                
                                print ""
        
                                self.arm_end_com[0] = self.arm_end[0] + x_addition
                                #  self.arm_end_com[0] = self.arm_end[0] + 0.030
                                self.arm_end_com[1] = self.arm_end[1]
                                self.arm_end_com[2] = self.arm_end[2]
        
                                print "next end position x : ", self.arm_end_com[0] + 0.270
                                print "next end position y : ", self.arm_end_com[1] + 0.000
                                print "next end position z : ", self.arm_end_com[2] + 0.935
                                
                                print ""
        
                                self.joint1_com = math.atan2(self.arm_end_com[1], self.arm_end_com[0])
        
                                I  = math.sqrt(self.arm_end_com[0]**2 + self.arm_end_com[1]**2)
                                #  print "I : ", I
                                I_L3off = self.L3 * math.cos(joint2_rad)
                                Z_L3off = self.L3 * math.sin(joint2_rad)
                                ld = math.sqrt((I - I_L3off)**2 + (self.arm_end_com[2] + (self.L1 + self.L2) - Z_L3off)**2)
                                #  print "ld : ", ld

                                joint5_com_limit = ((self.L4 + self.L5)**2 + self.L6**2 - ld**2) / (2 * (self.L4 + self.L5) * self.L6)
                                joint3_com_limit = ((self.L4 + self.L5)**2 + ld**2 - self.L6**2) / (2 * (self.L4 + self.L5) * ld)
                                
                                if (joint5_com_limit <= 1 and joint3_com_limit <= 1) and x_addition < 0.030:
                                    self.joint5_com =self.pi - math.acos(((self.L4 + self.L5)**2 + self.L6**2 - ld**2) / (2 * (self.L4 + self.L5) * self.L6))
            
                                    self.joint3_com = math.acos(((self.L4 + self.L5)**2 + ld**2 - self.L6**2) / (2 * (self.L4 + self.L5) * ld)) + math.atan2((self.arm_end_com[2] + (self.L1 + self.L2) - Z_L3off), (I - I_L3off)) - joint2_rad
                                    
                                    
                                    self.joint1_com = self.joint1_com / self.pi * 180.0
                                    self.joint3_com = self.joint3_com / self.pi * 180.0
                                    self.joint5_com = self.joint5_com / self.pi * 180.0
                                    print "joint1 command : ", self.joint1_com
                                    print "joint3 command : ", self.joint3_com
                                    print "joint5 command : ", self.joint5_com
                                    
                                    print ""
                                    
                                    #  if pub_11_flag:
                                        #  pub_11.publish(self.joint1_com)
                                        #  pub_11_flag = False
                                    #  elif pub_13_flag:
                                        #  pub_13.publish(self.joint3_com)
                                        #  pub_11_flag = True
                                        #  pub_13_flag = False
                                    #  else:
                                        #  pub_15.publish(self.joint5_com)
                                        #  pub_13_flag = True
                                    #  print "joint1, joint3, joint5  command publish!!!!"
            
                                    x_addition += 0.001
                                    print "x add : ", x_addition
                                else:
                                    x_addition = 0.0
                                    print "can not reach next end position!!!!"
                                    print "this episode finish!!!!!!!!!!!!!!"
                                    self.action_num = 0
                                    #  self.joint1 = self.init_next_joint1
                                    #  self.joint3 = self.init_next_joint3
                                    #  self.joint5 = self.init_next_joint5
                                    pub_1.publish(self.joint1)
                                    pub_3.publish(self.joint3)
                                    pub_5.publish(self.joint5)
                                    pub_6.publish(self.action_num)
                                    loss_list = []
        
                                    pub_9.publish(self.target_point)

                                    self.wait_flag = True
                                    
                                #  self.state_observation_flag = False
                                #  self.state_observation_flag1 = False
                                #  self.state_observation_flag3 = False
                                #  self.state_observation_flag5 = False

                                pub_15.publish(self.joint5_com)
                                print "joint5 command publish!! ----> ", self.joint5_com
                                pub_11.publish(self.joint1_com)
                                print "joint1 command publish!! ----> ", self.joint1_com
                                rospy.sleep(0.05)
                                pub_13.publish(self.joint3_com)
                                print "joint3 command publish!! ----> ", self.joint3_com
                loop_rate.sleep()
    
if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
