#!/usr/bin/env python
#coding:utf-8
import argparse

import rospy

import numpy as np
import chainer
import chainer.functions as F
from chainer import cuda, optimizers

from std_msgs.msg import Float64, Int64, Float32, UInt8, Bool
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from rospy.numpy_msg import numpy_msg
from q_update.msg import State

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
	
        init_joint1 = 0
        init_joint2 = -50
        init_joint3 = 105

        init_next_joint1 = init_joint1
        init_next_joint2 = init_joint2
        init_next_joint3 = init_joint3
        
        data_size = 5
        #  data_size = 10**5

	num_a = 3**3

	ALPHA = 0.01
	GAMMA = 0.9
        EPSILON = 1.0

	wait_flag = True
	#  wait_flag = False
	select_action_flag = False
	q_update_flag = False
	state_observation_flag = False
	state_observation_flag1 = False
	state_observation_flag2 = False
	state_observation_flag3 = False

        reward_flag = False

        L1 = 0.064
        L2 = 0.063
        L3 = 0.250
        L4 = 0.100
        L5 = 0.100
        L6 = 0.130


        joint1_rad = 0.0 / 180.0 * pi
        #  joint2_rad = -(1.515/self.pi*180.0 - 90.0) / 180.0 * self.pi
        joint2_rad_1 = -1 * (-80.0 / 180.0 * pi + pi / 2.0)
        joint2_rad_2 = -1 * (-60.0 / 180.0 * pi  + pi / 2.0)
        joint3_rad_1 = 55.0 / 180.0 * pi
        joint3_rad_2 = 95.0 / 180.0 * pi
        joint5_rad = 45.0 / 180.0 * pi
        L_1 = L3 * math.cos(joint2_rad_1) + (L4 + L5) * math.cos(joint2_rad_1 + joint3_rad_1) + L6 * math.cos(joint2_rad_1 + joint3_rad_1 - joint5_rad) - 0.010
        L_2 = L3 * math.cos(joint2_rad_2) + (L4 + L5) * math.cos(joint2_rad_2 + joint3_rad_2) + L6 * math.cos(joint2_rad_2 + joint3_rad_2 - joint5_rad) - 0.010
	
        def __init__(self):
		self.joint_state = np.zeros((3), dtype=np.float32)
		self.action_num = 0
		self.reward = 0.0

                self.target_point = PointCloud()
                self.target_init_y = 0.000
                #  self.target_init_x = 0.680
                self.target_init_x = math.sqrt(self.L_2**2 - self.target_init_y**2) + 0.270
                self.target_init_z = 0.900

		self.num_step = 0
                self.num_episode = 0

                self.model = chainer.FunctionSet(
                        l1 = F.Linear(6, 2560),
                        l2 = F.Linear(2560, 1280),
                        l3 = F.Linear(1280, 640),
                        l4 = F.Linear(640, 320),
                        l5 = F.Linear(320, 160),
                        l6 = F.Linear(160, 80),
                        l7 = F.Linear(80, 27, initialW=np.zeros((27, 80), dtype=np.float32)),
                        )
                if args.gpu >= 0:
                    self.model.to_gpu()
                self.optimizer = optimizers.SGD(self.ALPHA)
                self.optimizer.setup(self.model)
                self.q_list = chainer.Variable(xp.zeros((1, 27), dtype=xp.float32))

                self.action = 0
                self.joint_state[0] = self.init_joint1
                self.joint_state[1] = self.init_joint2
                self.joint_state[2] = self.init_joint3
                self.joint1 = self.init_joint1
                self.joint2 = self.init_joint2
                self.joint3 = self.init_joint3
                self.next_joint1 = self.init_next_joint1
                self.next_joint2 = self.init_next_joint2
                self.next_joint3 = self.init_next_joint3
                
                #history memory : D = [s(joint1, joint2, joint3, x, y, z), a, r, s_dash(joint1, joint2, joint3, x, y, z), end_episode_flag]
                self.D = [xp.zeros((self.data_size, 1, 6), dtype=xp.float32),
                          xp.zeros(self.data_size, dtype=xp.uint8),
                          xp.zeros((self.data_size, 1), dtype=xp.float32),
                          xp.zeros((self.data_size, 1, 6), dtype=xp.float32),
                          xp.zeros((self.data_size, 1), dtype=np.bool)]
                #  print self.D
                self.pub_time = rospy.Publisher("/time", Int64, queue_size = 1)
                self.pub_state = rospy.Publisher("/state", numpy_msg(State), queue_size = 1)
                self.pub_action = rospy.Publisher("/action", UInt8, queue_size = 1 )
                self.pub_reward = rospy.Publisher("/reward", Float32, queue_size = 1)
                self.pub_next_state = rospy.Publisher("/next_state", numpy_msg(State), queue_size = 1)
                self.pub_episode_end_flag = rospy.Publisher("/episode_end_flag", Bool, queue_size = 1)
	def state_observation_flag_callback(self, msg):
	    self.state_observation_flag = True
	    #  print "flag !!!"
        
        def joint1_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[0] = msg.data
            print "joint1_state : ",self.joint_state[0]
	    self.state_observation_flag1 = True
 
        def joint2_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[1] = msg.data
            print "joint2_state : ",self.joint_state[1]
	    self.state_observation_flag2 = True

        def joint3_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[2] = msg.data
            print "joint3_state : ",self.joint_state[2]
	    self.state_observation_flag3 = True        

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

        def cal_1_target_x(self, y_data):
            #  print "r : ", math.sqrt(self.L**2 - y_data**2) + 0.270
            return math.sqrt(self.L_1**2 - y_data**2) + 0.270

        def cal_2_target_x(self, y_data):
            #  print "r : ", math.sqrt(self.L**2 - y_data**2) + 0.270
            return math.sqrt(self.L_2**2 - y_data**2) + 0.270
        
        def my_norm(self, data):
            return xp.sqrt(xp.sum(data**2, axis=1))

        def forward(self, joint1_data, joint2_data, joint3_data):
            #  print "joint1(type) : ", type(joint1_data)
            #  print "joint2(type) : ", type(joint2_data)
            #  print "joint3(type) : ", type(joint3_data)
            joint1_data_float = float(joint1_data - self.mean(50.0, -50.0) ) / (50.0 - (-50.0))
            joint2_data_float = float(joint2_data - self.mean(0.0, -80.0)) / (0.0 - (-80.0))
            joint3_data_float = float(joint3_data - self.mean(150.0, 55.0) ) / (150.0 - (55.0))

            target_data_x = float((self.target_point.points[0].x - self.mean(self.cal_1_target_x(0.000), self.cal_2_target_x(0.000))) / (self.cal_1_target_x(0.000) - self.cal_2_target_x(0.000)))
            target_data_y = float((self.target_point.points[0].y - self.mean(0.160, -0.160 )) / (0.160 - (-0.160)))
            target_data_z = float((self.target_point.points[0].z - self.mean(1.000, 0.900)) / (1.000 - 0.900))
            
            x = chainer.Variable(xp.array([[joint1_data_float, joint2_data_float, joint3_data_float, target_data_x, target_data_y, target_data_z]], dtype=xp.float32))
            print "x :  ", x.data
            #  print "norm x(before) : ", self.my_norm(x.data)
            x.data = x.data / self.my_norm(x.data)
            #  print "norm x(after) : ", self.my_norm(x.data)
            print "x(nomal) : ", x.data
            y = None

            h1 = F.relu(self.model.l1(x))
            #  print "norm h1(before) : ", self.my_norm(h1.data)
            h1.data = h1.data / self.my_norm(h1.data)
            #  print "norm h1(after) : ", self.my_norm(h1.data)
            #  h1.data = h1.data /np.linalg.norm(h1.data)
            #  print "h1 : ", h1.data
            
            h2 = F.relu(self.model.l2(h1))
            #  print "norm h2(before) : ", self.my_norm(h2.data)
            h2.data = h2.data / self.my_norm(h2.data)
            #  print "norm h2(after) : ", self.my_norm(h2.data)
            #  h2.data = h2.data / np.linalg.norm(h2.data)
            #  print "h2 : ", h2.data
            
            h3 = F.relu(self.model.l3(h2))
            #  print "norm h3(before) : ", self.my_norm(h3.data)
            h3.data = h3.data / self.my_norm(h3.data)
            #  print "norm h3(after) : ", self.my_norm(h3.data)
            #  h3.data = h3.data / np.linalg.norm(h3.data)
            #  print "h3 : ", h3.data
            
            h4 = F.relu(self.model.l4(h3))
            #  print "norm h4(before) : ", self.my_norm(h4.data)
            h4.data = h4.data / self.my_norm(h4.data)
            #  print "norm h4(after) : ", self.my_norm(h4.data)
            #  h4.data = h4.data / np.linalg.norm(h4.data)
            #  print "h4 : ", h4.data
            
            h5 = F.relu(self.model.l5(h4)) 
            #  print "norm h5(before) : ", self.my_norm(h5.data)
            h5.data = h5.data / self.my_norm(h5.data)
            #  print "norm h5(after) : ", self.my_norm(h5.data)
            #  h5.data = h5.data / np.linalg.norm(h5.data)
            #  print "h5 : ", h5.data
            h6 = F.relu(self.model.l6(h5))
            #  print "norm h6(before) : ", self.my_norm(h6.data)
            h6.data = h6.data / self.my_norm(h6.data)
            #  print "norm h6(after) : ", self.my_norm(h6.data)
            #  h6.data = h6.data / np.linalg.norm(h6.data)
            #  print "h6 : ", h6.data
            y = self.model.l7(h6)
            print "y : ", y.data

            return y
            
        def stock_experience(self,time, state, action, reward, next_state, episode_end_flag):
            data_index = time % self.data_size
            #  print "state type : ", type(state)
            #  print "state : ", state
            #  print "action type : ", type(action)
            #  print "action : ", action
            #  print "reward type : ", type(reward)
            #  print "reward : ", reward
            #  print "next_state type : ", type(next_state)
            #  print "next_state : ", next_state
            #  print "episode_end_flag type : ", type(episode_end_flag)
            #  print "episode_end_flag : ", episode_end_flag
            self.D[0][data_index] = state
            self.D[1][data_index] = action
            self.D[2][data_index] = reward
            self.D[3][data_index] = next_state
            self.D[4][data_index] = episode_end_flag
            state_cpu = cuda.to_cpu(state)
            state_cpu = np.float32(state_cpu)
            #  print "state(cpu) : ", state_cpu
            next_state_cpu = cuda.to_cpu(next_state)
            next_state_cpu = np.float32(next_state_cpu)
            #  print "next_state(cpu) : ", next_state_cpu
            self.pub_time.publish(time)
            self.pub_state.publish(state_cpu)
            self.pub_action.publish(action)
            self.pub_reward.publish(reward)
            self.pub_next_state.publish(next_state_cpu)
            self.pub_episode_end_flag.publish(episode_end_flag)

	def select_action(self, joint1, joint2, joint3):
            self.q_list = self.forward(joint1, joint2, joint3)
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

	def epsilon_greedy(self, joint1, joint2, joint3):
	    if self.EPSILON > random():
		a = randint(0, self.num_a-1)
                print "random select!!"
	    else:
		a = self.select_action(joint1, joint2, joint3)
                print "argmax Q select!!"
	    return a
        
        def main(self):
            rospy.init_node('q_update_my_replay')
            
            rospy.Subscriber("/state_observation_flag", Int64, self.state_observation_flag_callback)
            rospy.Subscriber("/joint1_state", Float64, self.joint1_state_callback)
            rospy.Subscriber("/joint2_state", Float64, self.joint2_state_callback)
            rospy.Subscriber("/joint3_state", Float64, self.joint3_state_callback)

            
            pub_1 = rospy.Publisher("/joint1_pose", Float64, queue_size = 1)
            pub_2 = rospy.Publisher("/joint2_pose", Float64, queue_size = 1)
            pub_3 = rospy.Publisher("/joint3_pose", Float64, queue_size = 1)
            pub_6 = rospy.Publisher("/action_num", Int64, queue_size = 1)
            pub_7 = rospy.Publisher("/num_step", Int64, queue_size = 1)
            pub_8 = rospy.Publisher("/num_episode", Int64, queue_size = 1)

            pub_9 = rospy.Publisher("/target_point", PointCloud, queue_size = 1)
            pub_10 = rospy.Publisher("/vis_target_point", PointCloud, queue_size = 1)
            
            loop_rate = rospy.Rate(100)
            
            count = 0
            count_temp = 0

            print "joint1 : ", self.joint1
            print "joint2 : ", self.joint2
            print "joint3 : ", self.joint3

            print "next joint1 : ", self.next_joint1
            print "next joint2 : ", self.next_joint2
            print "next joint3 : ", self.next_joint3

            step_count = 0
            episode_count = 0
            
            time = 0

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
            print "target_point : ", self.target_point
            
            state = xp.array([self.next_joint1, self.next_joint2, self.next_joint3, self.target_point.points[0].x, self.target_point.points[0].y, self.target_point.points[0].z])
            #  print "state : "
            #  print state

            print "Q Learning Start!!"

            #  print "L_1 : ", self.L_1
            #  print "L_2 : ", self.L_2
            #  print "x : ", self.L_1 * math.cos(0.0)+0.270
            #  rand_target_vis_y = -0.08
            while not rospy.is_shutdown():
                if episode_count == 0:
                    if step_count == 0:    
                        pub_9.publish(self.target_point)
                #  if count == 0:
                    #  rand_target_vis_x = math.sqrt(self.L_1**2 - rand_target_vis_y**2) + 0.270
                #  elif count == 1:
                    #  rand_target_vis_x = math.sqrt(self.L_2**2 - rand_target_vis_y**2) + 0.270
                #  rand_target_vis_z = self.target_init_z
                
                #  rand_target_vis_y = uniform(self.target_init_y-0.06, self.target_init_y+0.06)
                #  rand_target_x = self.target_init_x
                #  rand_target_vis_x = uniform(math.sqrt(self.L_2**2 - rand_target_vis_y**2) + 0.270, math.sqrt(self.L_1**2 - rand_target_vis_y**2) + 0.270)
                #  rand_target_vis_z = self.target_init_z
                #  if rand_target_vis_y <=0.08:    
                    #  target_vis.points.append(Point32(rand_target_vis_x, rand_target_vis_y, rand_target_vis_z))
                    #  rospy.sleep(0.5)
                    #  pub_10.publish(target_vis)
                    #  rand_target_vis_y += 0.01
                    #  if count == 0:
                        #  rand_target_vis_y += 0.01
                    #  if count == 1:
                        #  rand_target_vis_y -= 0.01

                    #  if rand_target_vis_y == 0.08:
                        #  count += 1
                    #  if rand_target_vis_y == -0.08:
                        #  count += 1
                    
                if self.wait_flag:
                    print "wait 1 seconds!!"
                    count += 1
                    if count == 100:
                        self.wait_flag = False
                        self.select_action_flag = False
                        self.q_update_flag = False
                        self.state_observation_flag = True
                        self.state_observation_flag1 = True
                        self.state_observation_flag2 = True
                        self.state_observation_flag3 = True
                        count = 0
                    if count == 50:
                        self.action_num = 0
                        self.joint1 = self.init_next_joint1
                        self.joint2 = self.init_next_joint2
                        self.joint3 = self.init_next_joint3
                        self.reward = 0.0
                        pub_1.publish(self.joint1)
                        pub_2.publish(self.joint2)
                        pub_3.publish(self.joint3)
                        pub_6.publish(self.action_num)
                else:
                    if self.select_action_flag:
                        step_count += 1
                        time += 1
                        self.action = self.epsilon_greedy(self.joint1, self.joint2, self.joint3)
                        self.action_num = self.action
                        print "self.action_num : ", self.action_num
                        pub_1.publish(self.joint1) 
                        pub_2.publish(self.joint2) 
                        pub_3.publish(self.joint3) 
                        pub_6.publish(self.action_num)
                        self.select_action_flag = False

                    if self.state_observation_flag and self.state_observation_flag1 and self.state_observation_flag2 and self.state_observation_flag3:
                        print "self.joint_state[0] : ",self.joint_state[0]
                        print "self.joint_state[1] : ",self.joint_state[1]
                        print "self.joint_state[2] : ",self.joint_state[2]
                        
                        print "now joint1 : ", self.joint1
                        print "now joint2 : ", self.joint2
                        print "now joint3 : ", self.joint3
                        
                        self.next_joint1 = self.joint_state[0]
                        self.next_joint2 = self.joint_state[1]
                        self.next_joint3 = self.joint_state[2]
                        print "next joint1 : ", self.next_joint1
                        print "next joint2 : ", self.next_joint2
                        print "next joint3 : ", self.next_joint3

                        next_state = xp.array([self.next_joint1, self.next_joint2, self.next_joint3, self.target_point.points[0].x, self.target_point.points[0].y, self.target_point.points[0].z])
                        #  print "next_state : "
                        #  print next_state
                        
                        if step_count == 0:
                            self.select_action_flag = True
                        else:
                            self.reward = self.reward_calculation_client(step_count)
                            print "reward : ", self.reward
                            self.q_update_flag = True
                        self.state_observation_flag = False
                        self.state_observation_flag1 = False
                        self.state_observation_flag2 = False
                        self.state_observation_flag3 = False

                    if self.q_update_flag:
                        target_val = self.reward + self.GAMMA * np.max(self.forward(self.next_joint1, self.next_joint2, self.next_joint3).data)
                        self.optimizer.zero_grads()
                        tt = xp.copy(self.q_list.data)
                        tt[0][self.action] = target_val
                        target = chainer.Variable(tt)
                        #  loss = 0.5 * (target - self.q_list) ** 2
                        loss = F.mean_squared_error(target, self.q_list)
                        loss_list.append(np.max(loss.data))
                        loss.backward()
                        self.optimizer.update()

                        self.reward_flag = True
                        self.select_action_flag = True
                        self.q_update_flag = False
                        print "episode : %d " % episode_count,
                        print "step : %d " % step_count,
                        print "now joint1 : %d " % self.joint1,
                        print "now joint2 : %d " % self.joint2,
                        print "now joint3 : %d " % self.joint3,
                        print "now action : %d" % self.action,
                        print "loss : ", np.max(loss.data),
                        print "reward : %.1f  " % self.reward,
                        print "EPSILON : %.5f " % self.EPSILON
                        print ""

                    if self.reward_flag:
                        self.reward_flag = False
                        if self.reward >= 1:
                            print "episode : %d " % episode_count,
                            print "step : %d " % step_count,
                            print "now joint1 : %d " % self.joint1,
                            print "now joint2 : %d " % self.joint2,
                            print "now joint3 : %d " % self.joint3,
                            print "now action : %d" % self.action,
                            print "loss average : %.3f " % (sum(loss_list)/len(loss_list)),
                            print "reward : %.1f  " % self.reward,
                            print "EPSILON : %.5f " % self.EPSILON,
                            print "succsess!!"
                            print ""

                            temp_result = np.array(([[episode_count, step_count]]), dtype=np.int32)
                            if episode_count == 0:
                                test_result = temp_result
                            else:
                                test_result = np.r_[test_result, temp_result]

                            
                            if episode_count%100 == 0:
                                model_filename = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model_%d.dat" % episode_count
                                f = open(model_filename, 'w')
                                pickle.dump(self.model, f) 
                                filename_result1 = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_result_%d.txt" % episode_count
                                np.savetxt(filename_result1, test_result, fmt="%d", delimiter=",")
                            
                            self.stock_experience(time, state, self.action, self.reward, next_state, True)
                            print "self.D : "
                            print self.D

                            rand_target_y = uniform(self.target_init_y-0.06, self.target_init_y+0.06)
                            rand_target_x = math.sqrt(self.L_2**2 - rand_target_y**2) + 0.270
                            #  rand_target_x = uniform(math.sqrt(self.L_2**2 - rand_target_y**2) + 0.270, math.sqrt(self.L_1**2 - rand_target_y**2) + 0.270)
                            rand_target_z = self.target_init_z
                            self.target_point.header.stamp = rospy.Time.now()
                            self.target_point.points[0].x = rand_target_x
                            self.target_point.points[0].y = rand_target_y
                            self.target_point.points[0].z = rand_target_z
                            
                            
                            step_count = 0
                            episode_count += 1
                            episode_now = episode_count

                            self.action_num = 0
                            self.joint1 = self.init_next_joint1
                            self.joint2 = self.init_next_joint2
                            self.joint3 = self.init_next_joint3
                            pub_1.publish(self.joint1)
                            pub_2.publish(self.joint2)
                            pub_3.publish(self.joint3)
                            pub_6.publish(self.action_num)
                            loss_list = []

                            pub_9.publish(self.target_point)

                            self.wait_flag = True
                        else:
                            if step_count < 300:
                                self.stock_experience(time, state, self.action, self.reward, next_state, False)
                                print "self.D : "
                                print self.D

                                self.joint1 = self.next_joint1
                                self.joint2 = self.next_joint2
                                self.joint3 = self.next_joint3

                                state = next_state

                                episode_past = episode_now
                            else:
                                print "episode : %d " % episode_count,
                                print "step : %d " % step_count,
                                print "now joint1 : %d " % self.joint1,
                                print "now joint2 : %d " % self.joint2,
                                print "now joint3 : %d " % self.joint3,
                                print "now action : %d" % self.action,
                                print "loss average : %.3f " % (sum(loss_list)/len(loss_list)),
                                print "reward : %.1f  " % self.reward,
                                print "EPSILON : %.5f " % self.EPSILON,
                                print "failuer!!"
                                print ""

                                temp_result = np.array(([[episode_count, step_count]]), dtype=np.int32)
                                if episode_count == 0:
                                    test_result = temp_result
                                else:
                                    test_result = np.r_[test_result, temp_result]
                                
                                if episode_count%100 == 0:
                                    model_filename = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model_%d.dat" % episode_count
                                    f = open(model_filename, 'w')
                                    pickle.dump(self.model, f)
                                    filename_result1 = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_result_%d.txt" % episode_count
                                    np.savetxt(filename_result1, test_result, fmt="%d", delimiter=",")
                                    
                                self.stock_experience(time, state, self.action, self.reward, next_state, True)
                                print "self.D : "
                                print self.D
                            
                                rand_target_y = uniform(self.target_init_y-0.06, self.target_init_y+0.06)
                                rand_target_x = math.sqrt(self.L_2**2 - rand_target_y**2) + 0.270
                                #  rand_target_x = uniform(math.sqrt(self.L_2**2 - rand_target_y**2) + 0.270, math.sqrt(self.L_1**2 - rand_target_y**2) + 0.270)
                                rand_target_z = self.target_init_z
                                self.target_point.header.stamp = rospy.Time.now()
                                self.target_point.points[0].x = rand_target_x
                                self.target_point.points[0].y = rand_target_y
                                self.target_point.points[0].z = rand_target_z
                                
                                step_count = 0
                                episode_count += 1
                                episode_now = episode_count

                                self.action_num = 0
                                self.joint1 = self.init_next_joint1
                                self.joint2 = self.init_next_joint2
                                self.joint3 = self.init_next_joint3
                                pub_1.publish(self.joint1)
                                pub_2.publish(self.joint2)
                                pub_3.publish(self.joint3)
                                pub_6.publish(self.action_num)
                                loss_list = []

                                pub_9.publish(self.target_point)
                                

                                self.wait_flag = True
                        
                        if math.fabs(episode_now - episode_past) > 1e-6:
                            if self.EPSILON > 0.1000:
                                self.EPSILON -= 0.000025

                        self.num_step = step_count
                        pub_7.publish(self.num_step)
                        self.num_episode = episode_count
                        pub_8.publish(self.num_episode)
                        

                        if episode_count > 40000:
                            filename_result = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_result.txt"
                            np.savetxt(filename_result, test_result, fmt="%d", delimiter=",")
                            print "Finish!!!"
                            break
                
                loop_rate.sleep()
    
if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
