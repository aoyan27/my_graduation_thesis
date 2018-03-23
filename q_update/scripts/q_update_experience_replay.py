#!/usr/bin/env python
#coding:utf-8
import argparse

import rospy

import copy

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
	
        init_joint1 = 0
        init_joint2 = -50
        init_joint3 = 105

        init_next_joint1 = init_joint1
        init_next_joint2 = init_joint2
        init_next_joint3 = init_joint3
	
	num_a = 3**3

	ALPHA = 0.01
	GAMMA = 0.9
        #  EPSILON = 1.0
        EPSILON = 0.0

        wait_flag = False
	#  wait_flag = True
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
        
        initial_exploration = 1000
        replay_size = 32
        data_size = 1000
        target_model_update_freq = 10**2

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
                self.model_target = copy.deepcopy(self.model)
                if args.gpu >= 0:
                    self.model.to_gpu()
                    self.model_target.to_gpu()

                self.optimizer = optimizers.SGD(self.ALPHA)
                self.optimizer.setup(self.model)
                self.q_list = chainer.Variable(xp.zeros((1, 27), dtype=xp.float32))

                self.action = 0
                self.joint1 = self.init_joint1
                self.joint2 = self.init_joint2
                self.joint3 = self.init_joint3
                self.next_joint1 = self.init_next_joint1
                self.next_joint2 = self.init_next_joint2
                self.next_joint3 = self.init_next_joint3

                # history data : D = [s(joint1, joint2, joint3, goal_x, goal_y, goal_z), a, reward, s_dash(joint1', joint2', joint3', x_goal, y_goal, z_goal), end_episode_flag]
                self.D = [xp.zeros((self.data_size, 1, 6), dtype=xp.float32),
                          xp.zeros(self.data_size, dtype=xp.uint8),
                          xp.zeros((self.data_size, 1), dtype=xp.float32),
                          xp.zeros((self.data_size, 1, 6), dtype=xp.float32),
                          xp.zeros((self.data_size, 1), dtype=np.bool)]



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
        
        def forward(self, state, action, reward, next_state, episode_end):
            num_of_batch = state.shape[0]
            #  print "num_of_batch : ", num_of_batch
            s = chainer.Variable(state)
            #  print "s : "
            #  print s.data
            next_s = chainer.Variable(next_state)
            #  print "next_s : "
            #  print next_s.data

            Q = self.Q_func(s)
            #  print "Q : "
            #  print Q.data

            tmp = self.Q_func_target(next_s)

            tmp = list(map(np.max, tmp.data.get()))

            next_max_Q = xp.asanyarray(tmp, dtype=xp.float32)

            target = xp.asanyarray(Q.data.get(), dtype=xp.float32)
            #  print "target : "
            #  print target

            for i in xrange(num_of_batch):
                if not episode_end[i][0]:
                    tmp_ = reward[i] + self.GAMMA * next_max_Q[i]
                    #  print "tmp_(not) : ", tmp_
                else:
                    tmp_ = reward[i] + self.GAMMA * next_max_Q[i]
                    #  print "tmp_ : ", tmp_

                action_index = int(action[i][0])
                #  print "action[i][0] : ", action[i][0]
                #  print "action_index : ", action_index
                #  print type(action_index)
                #  print "target[0][0] : ", target[0][0]
                #  print "target[i][action_index] : ", target[i][action_index]
                target[i ,action_index] = tmp_
                #  print "target[i, action_index] : ", target[i, action_index]
                #  print "target[i][action_index] : ", target[i][action_index]

            td = chainer.Variable(target) - Q

            td_tmp = td.data + 1000.0 * (abs(td.data) <= 1)

            td_clip = td * (abs(td.data) <= 1) + td/abs(td_tmp) * (abs(td.data) > 1)
            #  print "td : "
            #  print td_clip.data

            zero_val = chainer.Variable(xp.zeros((self.replay_size, self.num_a), dtype=xp.float32))
            loss = F.mean_squared_error(td_clip, zero_val)

            return loss, Q

        def stock_experience(self, t, state, action, reward, next_state, episode_end_flag):
            data_index = t % self.data_size

            if episode_end_flag is True:
                self.D[0][data_index] = state
                self.D[1][data_index] = action
                self.D[2][data_index] = reward
                self.D[3][data_index] = next_state
            else:
                self.D[0][data_index] = state
                self.D[1][data_index] = action
                self.D[2][data_index] = reward
                self.D[3][data_index] = next_state
            self.D[4][data_index] = episode_end_flag
        
        def experience_replay(self, t):
            #  print "AAAAAAAAAAAAAAAAAAAAAAAAAAaa"
            #  print "t : ", t
            if self.initial_exploration < t:
                #  print "experience replay!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                if t < self.data_size:
                    replay_index = np.random.randint(0, t, (self.replay_size, 1))
                    #  print "replay_index : "
                    #  print replay_index
                else:
                    replay_index = np.random.randint(0, self.data_size, (self.replay_size, 1))
                    #  print "replay_index : "
                    #  print replay_index
                #  print "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
                s_replay = xp.ndarray(shape=(self.replay_size, 1, 6), dtype=xp.float32)
                a_replay = xp.ndarray(shape=(self.replay_size, 1), dtype=xp.uint8)
                r_replay = xp.ndarray(shape=(self.replay_size, 1), dtype=xp.float32)
                next_s_replay = xp.ndarray(shape=(self.replay_size, 1, 6), dtype=xp.float32)
                episode_end_replay = xp.ndarray(shape=(self.replay_size, 1), dtype=np.bool)

                for i in xrange(self.replay_size):
                    s_replay[i] = xp.asarray(self.D[0][replay_index[i][0]], dtype=xp.float32)
                    a_replay[i] = self.D[1][replay_index[i][0]]
                    r_replay[i] = self.D[2][replay_index[i][0]]
                    next_s_replay[i] = xp.asarray(self.D[3][replay_index[i][0]], dtype=xp.float32)
                    episode_end_replay[i] = self.D[4][replay_index[i][0]]
                
                #  print "self.D : "
                #  print self.D
                #  print "s_replay : "
                #  print s_replay
                #  print "a_replay : "
                #  print a_replay
                #  print "r_replay : "
                #  print r_replay
                #  print "next_s_replay : "
                #  print next_s_replay
                #  print "episode_end_replay : "
                #  print episode_end_replay

                self.optimizer.zero_grads()
                loss, _ = self.forward(s_replay, a_replay, r_replay, next_s_replay, episode_end_replay)
                loss.backward()
                self.optimizer.update()

                return loss

        def Q_func(self, state):
            h1 = F.relu(self.model.l1(state))
            h2 = F.relu(self.model.l2(h1))
            h3 = F.relu(self.model.l3(h2))
            h4 = F.relu(self.model.l4(h3))
            h5 = F.relu(self.model.l5(h4)) 
            h6 = F.relu(self.model.l6(h5))
            Q = self.model.l7(h6)
            return Q
        
        def Q_func_target(self, state):
            h1 = F.relu(self.model.l1(state))
            h2 = F.relu(self.model.l2(h1))
            h3 = F.relu(self.model.l3(h2))
            h4 = F.relu(self.model.l4(h3))
            h5 = F.relu(self.model.l5(h4)) 
            h6 = F.relu(self.model.l6(h5))
            Q = self.model.l7(h6)
            return Q


	def select_action(self, state):
            #  print type(state)
            s = chainer.Variable(state)
            self.q_list = self.Q_func(s)
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

	def epsilon_greedy(self, state):
            #  print "state : "
            #  print state
            s = chainer.Variable(state)
            Q_now = self.Q_func(s)
            Q = Q_now.data
            #  print "Q : "
            #  print Q
	    if self.EPSILON > random():
		a = randint(0, self.num_a-1)
                string = 'random select!!'
                #  print "random select!!"
	    else:
		a = self.select_action(state)
                string = 'argmax Q select!!'
                #  print "argmax Q select!!"
	    return a, Q, string
        
        def main(self):
            rospy.init_node('q_update_client_tis')
            
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
            
            time = -1

            step_count = 0
            episode_count = 0
            
            episode_now = 0
            episode_past = 0

            temp_count = 0

            loss_list = []

            success_time = 0
            falure_time = 0

            target_vis = PointCloud()
            target_vis.header.frame_id = "/base_link"
            target_vis.header.stamp = rospy.Time.now()

            self.target_point.header.frame_id = "/base_link"
            self.target_point.header.stamp = rospy.Time.now()
            self.target_point.points.append(Point32(self.target_init_x, self.target_init_y, self.target_init_z))
            #  print type(self.target_point.points[0].x)
            print "target_point : ", self.target_point
            
            state = xp.array([[self.joint1, self.joint2, self.joint3, self.target_point.points[0].x, self.target_point.points[0].y, self.target_point.points[0].z]])
            #  print state.dtype
            state = state.astype(xp.float32)
            #  print state.dtype
            #  print "state : "
            #  print type(state[0])

            #  print "Q Learning Start!!"
            string = 'deep Q learning start!'
            print string

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
                    if self.initial_exploration < time:
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
                        #  print "statte : ", state
                        self.action, Q_now, string = self.epsilon_greedy(state)
                        self.action_num = self.action
                        
                        #  if self.initial_exploration < time:
                            #  print "self.action_num : ", self.action_num
                            #  print string
                        
                        pub_1.publish(self.joint1) 
                        pub_2.publish(self.joint2) 
                        pub_3.publish(self.joint3) 
                        pub_6.publish(self.action_num)
                        self.select_action_flag = False

                    if self.state_observation_flag and self.state_observation_flag1 and self.state_observation_flag2 and self.state_observation_flag3:
                        #  if self.initial_exploration < time:
                            #  print "self.joint_state[0] : ",self.joint_state[0]
                            #  print "self.joint_state[1] : ",self.joint_state[1]
                            #  print "self.joint_state[2] : ",self.joint_state[2]
                            
                            #  print "now joint1 : ", self.joint1
                            #  print "now joint2 : ", self.joint2
                            #  print "now joint3 : ", self.joint3
                        
                        #  print "state : "
                        #  print state

                        self.next_joint1 = self.joint_state[0]
                        self.next_joint2 = self.joint_state[1]
                        self.next_joint3 = self.joint_state[2]

                        #  if self.initial_exploration < time:
                            #  print "next joint1 : ", self.next_joint1
                            #  print "next joint2 : ", self.next_joint2
                            #  print "next joint3 : ", self.next_joint3
                    
                        next_state = xp.array([[self.next_joint1, self.next_joint2, self.next_joint3, self.target_point.points[0].x, self.target_point.points[0].y, self.target_point.points[0].z]])
                        next_state = next_state.astype(xp.float32)
                        #  print "next_state : "
                        #  print next_state
                        
                        if step_count == 0:
                            self.select_action_flag = True
                        else:
                            self.reward = self.reward_calculation_client(step_count)
                            #  if self.initial_exploration < time:
                                #  print "reward : ", self.reward
                            self.reward_flag = True
                        self.state_observation_flag = False
                        self.state_observation_flag1 = False
                        self.state_observation_flag2 = False
                        self.state_observation_flag3 = False


                    if self.reward_flag:
                        self.reward_flag = False
                        self.select_action_flag = True
                        if self.reward >= 1:
                            self.stock_experience(time, state, self.action, self.reward, next_state, True)
                            #  print "self.D(+1) : "
                            #  print self.D

                            loss = self.experience_replay(time)
                            
                            success_time += 1
                            
                            vis_state = xp.array([int(state[0][0]), int(state[0][1]), int(state[0][2])])
                            vis_next_state = xp.array([int(next_state[0][0]),int(next_state[0][1]), int(next_state[0][2])])
                            
                            if self.initial_exploration < time:
                                print "episode : %d " % episode_count,
                                print "step : %d " % step_count,
                                print "time : %d " % time,
                                print "now state : %s " % vis_state,
                                print "next state : %s " % vis_next_state,
                                print "now action : %d (%s) " % (self.action, string),
                                print "Q_MAX : %f " % np.max(Q_now.get()),
                                print "reward : %.1f  " % self.reward,
                                print "EPSILON : %f " % self.EPSILON,
                                print "succsess!!",
                                print "success times : %d " % success_time,
                                print "falure times : %d " % falure_time

                                print "state : "
                                print state
                                print "Q : "
                                print Q_now
                                print ""

                            temp_result = np.array(([[episode_count, step_count]]), dtype=np.int32)
                            if episode_count == 0:
                                test_result = temp_result
                            else:
                                test_result = np.r_[test_result, temp_result]

                            
                            if episode_count%10000 == 0:
                                model_filename = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model_%d.dat" % episode_count
                                f = open(model_filename, 'w')
                                pickle.dump(self.model, f) 
                                filename_result1 = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_result.txt"
                                np.savetxt(filename_result1, test_result, fmt="%d", delimiter=",")
                            


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
                            
                            init_state = xp.array([[self.init_next_joint1, self.init_next_joint2, self.init_next_joint3,\
                                    self.target_point.points[0].x, self.target_point.points[0].y, self.target_point.points[0].z]])
                            init_state = init_state.astype(np.float32)
                            #  print "next_init_state : ", init_state
                            state = init_state

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
                                #  print "self.D : "
                                #  print self.D
                                
                                loss = self.experience_replay(time)
                                
                                vis_state = xp.array([int(state[0][0]), int(state[0][1]), int(state[0][2])])
                                vis_next_state = xp.array([int(next_state[0][0]),int(next_state[0][1]), int(next_state[0][2])])
                                    

                                if self.initial_exploration < time:
                                    print "episode : %d " % episode_count,
                                    print "step : %d " % step_count,
                                    print "time : %d " % time,
                                    print "now state : %s " % vis_state,
                                    print "next state : %s " % vis_next_state,
                                    print "now action : %d (%s) " % (self.action, string),
                                    print "Q_MAX : %f " % np.max(Q_now.get()),
                                    print "reward : %.1f " % self.reward,
                                    print "EPSILON : %f " % self.EPSILON
                                    
                                    print "state : "
                                    print state
                                    print "Q : "
                                    print Q_now
                                    print ""

                                
                                self.joint1 = self.next_joint1
                                self.joint2 = self.next_joint2
                                self.joint3 = self.next_joint3
                                
                                state = next_state

                                episode_past = episode_now
                            else:
                                self.stock_experience(time, state, self.action, self.reward, next_state, True)
                                #  print "self.D(-1) : "
                                #  print self.D
                                
                                loss = self.experience_replay(time)
                                
                                falure_time += 1
                                
                                vis_state = xp.array([int(state[0][0]), int(state[0][1]), int(state[0][2])])
                                vis_next_state = xp.array([int(next_state[0][0]),int(next_state[0][1]), int(next_state[0][2])])

                                if self.initial_exploration < time:
                                    print "episode : %d " % episode_count,
                                    print "step : %d " % step_count,
                                    print "time : %d " % time,
                                    print "now state : %s " % vis_state,
                                    print "next state : %s " % vis_next_state,
                                    print "now action : %d (%s) " % (self.action, string),
                                    print "Q_MAX : %f " % np.max(Q_now.get()),
                                    print "reward : %.1f  " % self.reward,
                                    print "EPSILON : %f " % self.EPSILON,
                                    print "failure!!"
                                    print "success times : %d " % success_time,
                                    print "falure times : %d " % falure_time
                                    
                                    print "state : "
                                    print state
                                    print "Q : "
                                    print Q_now
                                    print ""

                                temp_result = np.array(([[episode_count, step_count]]), dtype=np.int32)
                                if episode_count == 0:
                                    test_result = temp_result
                                else:
                                    test_result = np.r_[test_result, temp_result]
                                
                                if episode_count%10000 == 0:
                                    model_filename = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model_%d.dat" % episode_count
                                    f = open(model_filename, 'w')
                                    pickle.dump(self.model, f)
                                    filename_result1 = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_result.txt"
                                    np.savetxt(filename_result1, test_result, fmt="%d", delimiter=",")
                                    
                                
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
                                
                                init_state = xp.array([[self.init_next_joint1, self.init_next_joint2, self.init_next_joint3,\
                                        self.target_point.points[0].x, self.target_point.points[0].y, self.target_point.points[0].z]])
                                init_state = init_state.astype(np.float32)
                                #  print "next_init_state : ", init_state
                                state = init_state
                                
                                pub_1.publish(self.joint1)
                                pub_2.publish(self.joint2)
                                pub_3.publish(self.joint3)
                                pub_6.publish(self.action_num)
                                loss_list = []

                                pub_9.publish(self.target_point)
                                

                                self.wait_flag = True
                        
                        if self.initial_exploration < time:
                            #  self.EPSILON -= 0.000025
                            self.EPSILON -= 1.0 / 10**6
                            if self.EPSILON < 0.1000:
                                self.EPSILON = 0.1
                        else:
                            print "Initial_Exploration : %d/%d steps!!" % (time, self.initial_exploration)
                            self.EPSILON = 1.0

                        self.num_step = step_count
                        pub_7.publish(self.num_step)
                        self.num_episode = episode_count
                        pub_8.publish(self.num_episode)
                        

                        if episode_count > 500000 or time > 5000000:
                            filename_result = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_result_experience_replay.txt"
                            np.savetxt(filename_result, test_result, fmt="%d", delimiter=",")
                            print "Finish!!!"
                            break

                loop_rate.sleep()
    
if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
