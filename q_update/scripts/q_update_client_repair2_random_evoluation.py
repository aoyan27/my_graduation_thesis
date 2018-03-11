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
 
parser = argparse.ArgumentParser(description='q_update_client_repair')
parser.add_argument('--gpu', '-g', default=-1, type=int,
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
        init_joint3 = 95
        init_joint5 = 45

        init_next_joint1 = init_joint1
        init_next_joint3 = init_joint3
        init_next_joint5 = init_joint5
	
	num_a = 3**3

	ALPHA = 0.5
	GAMMA = 0.9
        EPSILON = 0.0

	wait_flag = False
	select_action_flag = False
	q_update_flag = False
	state_observation_flag = False
	state_observation_flag1 = False
	state_observation_flag3 = False
	state_observation_flag5 = False

        model_load_flag = False

        L1 = 0.064
        L2 = 0.063
        L3 = 0.250
        L4 = 0.100
        L5 = 0.100
        L6 = 0.130


        joint1_rad = 0.0 / 180.0 * pi
        joint2_rad = -1 * (-1.515 + pi / 2.0)
        joint3_rad = 55.0 / 180.0 * pi
        joint5_rad = 45.0 / 180.0 * pi
        L = L3 * math.cos(joint2_rad) + (L4 + L5) * math.cos(joint2_rad + joint3_rad) + L6 * math.cos(joint2_rad + joint3_rad - joint5_rad) - 0.010
	
        def __init__(self):
		self.joint_state = np.zeros((3), dtype=np.float32)
		self.action_num = 0
		self.reward = 0.0

                self.target_point = PointCloud()
                self.target_init_y = -0.080
                #  self.target_init_x = math.sqrt(self.L**2 - self.target_init_y**2) + 0.270
                self.target_init_x = self.L + 0.270
                self.target_init_z = 0.960

		self.num_step = 0
                self.num_episode = 0


                #  self.model = chainer.FunctionSet(
                        #  l1 = F.Linear(6, 2048),
                        #  l2 = F.Linear(2048, 1024),
                        #  l3 = F.Linear(1024, 512),
                        #  l4 = F.Linear(512, 256),
                        #  l5 = F.Linear(256, 128),
                        #  l6 = F.Linear(128, 64),
                        #  l7 = F.Linear(64, 27, initialW=np.zeros((27, 64), dtype=np.float32)),
                        #  )
                self.model_num = 0
                self.modelname = '/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_test16_angeal/dqn_arm_model_%d.dat' % self.model_num
                f = open(self.modelname, 'rb')
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


	def state_observation_flag_callback(self, msg):
	    self.state_observation_flag = True
	    #  print "flag !!!"
        
        def joint1_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[0] = msg.data
            #  print "joint1_state : ",self.joint_state[0]
	    self.state_observation_flag1 = True
 
        def joint3_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[1] = msg.data
            #  print "joint3_state : ",self.joint_state[1]
	    self.state_observation_flag3 = True

        def joint5_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[2] = msg.data
            #  print "joint5_state : ",self.joint_state[2]
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
            #  print "q_list : ", self.q_list.data
            Q_MAX = np.max(self.q_list.data.get())
            #  print "Q_MAX : ", Q_MAX
	    return a, Q_MAX

	def epsilon_greedy(self, joint1, joint3, joint5):
	    if self.EPSILON > random():
		a = randint(0, self.num_a-1)
                print "random select!!"
	    else:
		a = self.select_action(joint1, joint3, joint5)
                print "argmax Q select!!"
	    return a
        
        def main(self):
            rospy.init_node('q_update_client_repair2_random_demo')
            
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
            
            loop_rate = rospy.Rate(100)
            
            filename_result = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_evoluation.txt"

            count = 0
            count_temp = 0

            self.joint1 = self.init_joint1
            self.joint3 = self.init_joint3
            self.joint5 = self.init_joint5
            print "current joint1 : ", self.joint1
            print "current joint3 : ", self.joint3
            print "current joint5 : ", self.joint5

            self.next_joint1 = self.init_joint1
            self.next_joint3 = self.init_joint3
            self.next_joint5 = self.init_joint5
            #  print "next joint1 : ", self.next_joint1
            #  print "next joint3 : ", self.next_joint3
            #  print "next joint5 : ", self.next_joint5
            
            temp_Q_MAX = 0.0
            Q_MAX_list = np.array([])
            average_Q_MAX_list = []
            average_Q_MAX = 0.0

            step_count = 0
            step_count_list = []
            step_count_average = 0.0

            success_count = 0
            success_rate = 0.0
            success_rate_average = 0.0
            
            trial_count = 1
            
            #  loss_list = []

            reward_list = []
            sum_reward_average = 0.0
            reward_average = 0.0
            reward_average_list = []

            target_vis = PointCloud()
            target_vis.header.frame_id = "/base_link"
            target_vis.header.stamp = rospy.Time.now()

            self.target_point.header.frame_id = "/base_link"
            self.target_point.header.stamp = rospy.Time.now()
            self.target_point.points.append(Point32(self.target_init_x, self.target_init_y, self.target_init_z))
            print type(self.target_point.points[0].x)
            print "target_point : ", self.target_point

            print "Q Learning Start!!"

            #  print "L : ", self.L
            #  print "x : ", self.L * math.cos(0.0)+0.270
            #  rand_target_vis_y = -0.08
            while not rospy.is_shutdown():
                if self.model_load_flag:
                    self.modelname = '/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_test16_angeal/dqn_arm_model_%d.dat' % self.model_num
                    f = open(self.modelname, 'rb')
                    self.model = pickle.load(f)
                    print "Loard model number %d!!" % self.model_num
                    print "now loading..."
                if args.gpu >= 0:
                    self.model.to_gpu()
                self.optimizer = optimizers.SGD()
                self.optimizer.setup(self.model)
                self.model_load_flag = False

                if trial_count == 1:
                    if step_count == 0:
                        self.target_point.points[0].x = self.target_init_x
                        self.target_point.points[0].y = self.target_init_y
                        self.target_point.points[0].z = self.target_init_z
                        rand_target_x = self.target_init_x
                        rand_target_y = self.target_init_y
                        rand_target_z = self.target_init_z
                        pub_9.publish(self.target_point)
                #  rand_target_vis_x = math.sqrt(self.L**2 - rand_target_vis_y**2) + 0.270
                #  rand_target_vis_z = self.target_init_z
                
                #  if rand_target_vis_y <=0.08:    
                    #  target_vis.points.append(Point32(rand_target_vis_x, rand_target_vis_y, rand_target_vis_z))
                    #  pub_10.publish(target_vis)
                    #  rand_target_vis_y += 0.01 
                
                if self.wait_flag:
                    print "wait 1 seconds!!"
                    count += 1
                    if count == 100:
                        self.wait_flag = False
                        self.select_action_flag = False
                        self.q_update_flag = False
                        self.state_observation_flag = True
                        self.state_observation_flag1 = True
                        self.state_observation_flag3 = True
                        self.state_observation_flag5 = True
                        count = 0
                    if count == 10:
                        self.action_num = 0
                        self.joint1 = self.init_next_joint1
                        self.joint3 = self.init_next_joint3
                        self.joint5 = self.init_next_joint5
                        self.reward = self.reward_calculation_client(step_count)
                        self.reward = 0.0
                        pub_1.publish(self.joint1)
                        pub_3.publish(self.joint3)
                        pub_5.publish(self.joint5)
                        pub_6.publish(self.action_num)
                else:
                    if self.select_action_flag:
                        step_count += 1
                        self.action, temp_Q_MAX = self.epsilon_greedy(self.joint1, self.joint3, self.joint5)
                        self.action_num = self.action
                        #  print "self.action_num : ", self.action_num
                        Q_MAX_list = np.append(Q_MAX_list, np.array([temp_Q_MAX]))
                        #  print "Q_MAX_LIST : ", Q_MAX_list
                        pub_1.publish(self.joint1) 
                        pub_3.publish(self.joint3) 
                        pub_5.publish(self.joint5) 
                        pub_6.publish(self.action_num)
                        self.select_action_flag = False

                    if self.state_observation_flag and self.state_observation_flag1 and self.state_observation_flag3 and self.state_observation_flag5:
                        #  print "self.joint_state[0] : ",self.joint_state[0]
                        #  print "self.joint_state[1] : ",self.joint_state[1]
                        #  print "self.joint_state[2] : ",self.joint_state[2]
                        
                        print "now joint1 : ", self.joint1
                        print "now joint3 : ", self.joint3
                        print "now joint5 : ", self.joint5
                        
                        self.next_joint1 = self.joint_state[0]
                        self.next_joint3 = self.joint_state[1]
                        self.next_joint5 = self.joint_state[2]
                        print "next joint1 : ", self.next_joint1
                        print "next joint3 : ", self.next_joint3
                        print "next joint5 : ", self.next_joint5

                        if step_count == 0:
                            self.select_action_flag = True
                        else:
                            self.reward = self.reward_calculation_client(step_count)
                            reward_list.append(self.reward)
                            print "reward : ", self.reward
                            self.q_update_flag = True
                        self.state_observation_flag = False
                        self.state_observation_flag1 = False
                        self.state_observation_flag3 = False
                        self.state_observation_flag5 = False

                    if self.q_update_flag:
                        #  target_val = self.reward + self.GAMMA * np.max(self.forward(self.next_joint1, self.next_joint3, self.next_joint5).data)
                        #  self.optimizer.zero_grads()
                        #  tt = xp.copy(self.q_list.data)
                        #  tt[0][self.action] = target_val
                        #  target = chainer.Variable(tt)
                        #  loss = 0.5 * (target - self.q_list) ** 2
                        #  loss = F.mean_squared_error(target, self.q_list)
                        #  self.ALPHA = float(self.ALPHA)
                        #  loss.grad = xp.array([[self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA]], dtype=xp.float32)
                        #  loss_list.append(np.max(loss.data))
                        #  loss.backward()
                        #  self.optimizer.update()

                        self.select_action_flag = True
                        self.q_update_flag = False
                        print "model : %d " % self.model_num,
                        print "trial : %d " % trial_count,
                        print "step : %d " % step_count,
                        print "now joint1 : %d " % self.joint1,
                        print "now joint3 : %d " % self.joint3,
                        print "now joint5 : %d " % self.joint5,
                        print "now action : %d" % self.action,
                        #  print "loss : ", np.max(loss.data),
                        print "reward : %.1f  " % self.reward,
                        print "EPSILON : %.5f " % self.EPSILON
                        print ""


                        if self.reward >= 1:
                            print "model : %d " % self.model_num,
                            print "trial : %d " % trial_count,
                            print "step : %d " % step_count,
                            print "now joint1 : %d " % self.joint1,
                            print "now joint3 : %d " % self.joint3,
                            print "now joint5 : %d " % self.joint5,
                            print "now action : %d" % self.action,
                            #  print "loss average : %.3f " % (sum(loss_list)/len(loss_list)),
                            print "reward : %.1f  " % self.reward,
                            print "EPSILON : %.5f " % self.EPSILON,
                            print "success!!"
                            print ""
    
                            average_Q_MAX_list.append(np.average(Q_MAX_list))
                            #  print "average_Q_MAX_list : ", average_Q_MAX_list
                            Q_MAX_list = np.array([])
                            #  print "q_max_list : ", Q_MAX_list
                            
                            average_Q_MAX = sum(average_Q_MAX_list) / len(average_Q_MAX_list)
                            #  print "average_Q_MAX : ", average_Q_MAX

                            success_count += 1
                            success_rate = float(success_count) / float(trial_count)
                            #  print "success_count : ", success_count
                            #  print "trial_count : " ,trial_count
                            #  print "success_rate : ", success_rate
    
                            #  print "reward_list : ", reward_list
                            sum_reward_average += sum(reward_list) / (len(reward_list) - 1)
                            #  print "reward_list average : ", sum(reward_list) / len(reward_list)
                            reward_average = sum_reward_average / trial_count
                            #  print "reward_average : ", reward_average
                                
                            step_count_list.append(step_count)
                            #  print "step_count_list : ", step_count_list
                            step_count_average = sum(step_count_list) / len(step_count_list)
                            #  print "step_count_average : ", step_count_average
    
                            if trial_count == 100:
                                temp_result = np.array(([[float(self.model_num), step_count_average, reward_average, success_rate, average_Q_MAX]]))
                                if self.model_num == 0:
                                    test_result = temp_result
                                    print "test_result : ", test_result
                                else:
                                    test_result = np.r_[test_result, temp_result]
                                    print "test_result : ", test_result
                                self.model_load_flag = True
                                trial_count = 0
                                success_count = 0
                                sum_reward_average = 0.0
                                step_count_list = []
                                average_Q_MAX_list = []

                                if self.model_num == 30000:
                                    np.savetxt(filename_result, test_result, fmt="%.6f", delimiter=",")
                                    print "finish!!!"
                                    break
                                if self.model_num < 5000:
                                    self.model_num += 1000
                                else:
                                    self.model_num += 500

                            rand_target_y += math.fabs(2*self.target_init_y) / (100.0 - 1.0)
                            #  print "rand_traget_y : ", rand_target_y
                            #  rand_target_x = math.sqrt(self.L**2 - rand_target_y**2) + 0.270 
                            rand_target_x = self.L + 0.270 
                            rand_target_z = self.target_init_z
                            self.target_point.header.stamp = rospy.Time.now()
                            self.target_point.points[0].x = rand_target_x
                            self.target_point.points[0].y = rand_target_y
                            self.target_point.points[0].z = rand_target_z
                            
                            step_count = 0
                            trial_count += 1
    
                            self.action_num = 0
                            self.joint1 = self.init_next_joint1
                            self.joint3 = self.init_next_joint3
                            self.joint5 = self.init_next_joint5
                            pub_1.publish(self.joint1)
                            pub_3.publish(self.joint3)
                            pub_5.publish(self.joint5)
                            pub_6.publish(self.action_num)
                            
                            loss_list = []
                            reward_list = []
    
                            pub_9.publish(self.target_point)
                            
                            self.wait_flag = True

                        else:
                            if step_count < 70:
    
                                self.joint1 = self.next_joint1
                                self.joint3 = self.next_joint3
                                self.joint5 = self.next_joint5
                                print "step_count : ", step_count
    
                            else:
                                print "model : %d " % self.model_num,
                                print "trial : %d " % trial_count,
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
                                
                                average_Q_MAX_list.append(np.average(Q_MAX_list))
                                #  print "average_Q_MAX_list : ", average_Q_MAX_list
                                Q_MAX_list = np.array([])
                                #  print "q_max_list : ", Q_MAX_list
                                
                                average_Q_MAX = sum(average_Q_MAX_list) / len(average_Q_MAX_list)
                                #  print "average_Q_MAX : ", average_Q_MAX

                                success_rate = float(success_count) / float(trial_count)
                                #  print "success_count : ", success_count
                                #  print "trial_count : " ,trial_count
                                #  print "success_rate : ", success_rate
                                
                                #  print "reward_list : ", reward_list
                                sum_reward_average += sum(reward_list) / (len(reward_list) - 1)
                                #  print "reward_list average : ", sum(reward_list) / len(reward_list)
                                reward_average = sum_reward_average / trial_count
                                #  print "reward_average : ", reward_average
    
                                step_count_list.append(step_count)
                                #  print "step_count_list : ", step_count_list
                                step_count_average = sum(step_count_list) / len(step_count_list)
                                #  print "step_count_average : ", step_count_average
                                
                                if trial_count == 100:
                                    temp_result = np.array(([[float(self.model_num), step_count_average, reward_average, success_rate, average_Q_MAX]]))
                                    if self.model_num == 0:
                                        test_result = temp_result
                                        print "test_result : ", test_result
                                    else:
                                        test_result = np.r_[test_result, temp_result]
                                        print "test_result : ", test_result
                                    self.model_load_flag = True
                                    trial_count = 0
                                    success_count = 0
                                    sum_reward_average = 0.0
                                    step_count_list = []
                                    average_Q_MAX_list = []

                                    if self.model_num == 30000:
                                        np.savetxt(filename_result, test_result, fmt="%.6f", delimiter=",")
                                        print "finish!!!"
                                        break
                                    if self.model_num < 5000:
                                        self.model_num += 1000
                                    else:
                                        self.model_num += 500
                                
                                rand_target_y += math.fabs(2*self.target_init_y) / (100.0 - 1.0)
                                print "rand_target_y : ", rand_target_y
                                #  rand_target_x = math.sqrt(self.L**2 - rand_target_y**2) + 0.270 
                                rand_target_x = self.L + 0.270 
                                rand_target_z = self.target_init_z
                                self.target_point.header.stamp = rospy.Time.now()
                                self.target_point.points[0].x = rand_target_x
                                self.target_point.points[0].y = rand_target_y
                                self.target_point.points[0].z = rand_target_z
                                
                                step_count = 0
                                trial_count += 1
                                
                                self.action_num = 0
                                self.joint1 = self.init_next_joint1
                                self.joint3 = self.init_next_joint3
                                self.joint5 = self.init_next_joint5
                                pub_1.publish(self.joint1)
                                pub_3.publish(self.joint3)
                                pub_5.publish(self.joint5)
                                pub_6.publish(self.action_num)
                                
                                loss_list = []
                                reward_list = []
                                
                                pub_9.publish(self.target_point)
    
                                self.wait_flag = True
                                

                    self.num_step = step_count
                    pub_7.publish(self.num_step)
                    self.num_episode = trial_count
                    pub_8.publish(self.num_episode)
                    
                
                loop_rate.sleep()
    
if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
