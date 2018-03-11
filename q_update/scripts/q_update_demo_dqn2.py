#!/usr/bin/env python
#coding:utf-8
import argparse

import rospy

import numpy as np
import chainer
import chainer.functions as F
from chainer import cuda, optimizers

from std_msgs.msg import Float64, Int64
from sensor_msgs.msg import JointState

import math
from random import random, randint

from joint_to_s.srv import *
from reward_calculation.srv import *

import pickle
 
parser = argparse.ArgumentParser(description='q_update_demo_dqn')
parser.add_argument('--gpu', '-g', default=-1, type=int,
                    help='GPU ID (negative value indicates CPU)')
args = parser.parse_args()

if args.gpu >=0:
    cuda.check_cuda_available()
xp = cuda.cupy if args.gpu >=0 else np

class agent:
	pi = 3.141592
	init_state_joint1 = -35
	init_state_joint3 = 40
	init_state_joint5 = 30

	init_state = 17500
        init_next = 17500


	num_s = 70*70*10
	num_a = 3*3*3

	ALPHA = 0.5
	GAMMA = 0.9
        EPSILON = 0.0

	wait_flag = False
	select_action_flag = False
	q_update_flag = False
	state_observation_flag = False
	state_observation_flag1 = False
	state_observation_flag2 = False
	state_observation_flag3 = False

	def __init__(self):
		self.joint_state = np.zeros((3), dtype=np.float32)
		#  print self.joint_state
		self.action_num = 0
		self.reward = 0.0

		self.num_state = 0
		self.num_step = 0
                self.num_episode = 0

		self.state = 0
		self.next_state = 0
                
                f = open('/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_test13_dedede/dqn_arm_model_7950.dat', 'rb')
                self.model = pickle.load(f)
                if args.gpu >= 0:
                    self.model.to_gpu()
                self.optimizer = optimizers.SGD()
                self.optimizer.setup(self.model)
                self.q_list = chainer.Variable(xp.zeros((1, 27), dtype=xp.float32))

                self.action = 0
                self.state = 0
                self.next_state = 0
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
            print "joint1_state : ",self.joint_state[0]
	    self.state_observation_flag1 = True

        def joint3_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[1] = msg.data
            print "joint3_state : ",self.joint_state[1]
	    self.state_observation_flag2 = True

        def joint5_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[2] = msg.data
            print "joint5_state : ",self.joint_state[2]
	    self.state_observation_flag3 = True
 
        def joint_to_s_client(self, joint1_state, joint3_state, joint5_state):
            #  rospy.wait_for_service('state_num')
            req_joint1 = joint1_state
            req_joint3 = joint3_state
            req_joint5 = joint5_state
            try:
                joint_to_s = rospy.ServiceProxy('state_num', joint_state)
                resp = joint_to_s(req_joint1, req_joint3, req_joint5)
                #  print "state : ", resp.state
                return resp.state
            except rospy.ServiceException, e:
                print "Service call faild : %s" % e

        def reward_calculation_client(self, req_reward):
            #  rospy.wait_for_service('reward')
            try:
                reward_calculation = rospy.ServiceProxy('reward', reward)
                resp = reward_calculation(req_reward)
                #  print "reward : ", resp.reward
                return resp.reward
            except rospy.ServiceException, e:
                print "Service call faild : %s" % e

        def my_norm(self, *x):
            sum_temp = 0.0
            if args.gpu >=0:
                cuda.to_gpu(sum_temp)
            for i in range(len(x[0][0])):
                sum_temp += x[0][0][i]**2

            return math.sqrt(sum_temp)
            
        def forward(self, joint1_data, joint3_data, joint5_data):
            joint1_data_float = float(joint1_data - 35) /70.0
            joint3_data_float = float(joint3_data - 35) / 70.0
            joint5_data_float = float(joint5_data - 5) / 10.0
            #  joint1_data_float = float(self.joint_state[0])
            #  joint3_data_float = float(self.joint_state[1])
            #  joint5_data_float = float(self.joint_state[2])
            #  action_data_float = float(self.action) / 27.0

            x = chainer.Variable(xp.array([[joint1_data_float, joint3_data_float, joint5_data_float]], dtype=xp.float32))
            print "x :  ", x.data
            #  x.data = x.data / xp.linalg.norm(x.data)
            #  print "x(nomal) : ", x.data
            y = None

            h1 = F.relu(self.model.l1(x))
            #  h1.data = h1.data / self.my_norm(h1.data)
            #  h1.data = h1.data / xp.linalg.norm.trace(h1.data)
            #  print "h1 : ", h1.data
            h2 = F.relu(self.model.l2(h1))
            #  h2.data = h2.data / self.my_norm(h2.data)
            #  h2.data = h2.data / xp.linalg.norm.trace(h2.data)
            #  print "h2 : ", h2.data
            h3 = F.relu(self.model.l3(h2))
            #  h3.data = h3.data / self.my_norm(h3.data)
            #  h3.data = h3.data / xp.linalg.norm.trace(h3.data)
            #  print "h3 : ", h3.data
            h4 = F.relu(self.model.l4(h3))
            #  h4.data = h4.data / self.my_norm(h4.data)
            #  h4.data = h4.data / xp.linalg.norm.trace(h4.data)
            h5 = F.relu(self.model.l5(h4))
            #  h5.data = h5.data / self.my_norm(h5.data)
            #  h5.data = h5.data / xp.linalg.norm.trace(h5.data)
            #  h6 = F.relu(self.model.l6(h5))
            y = self.model.l6(h5)
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
            rospy.init_node('q_update_demo_dqn')
            
            rospy.Subscriber("/state_observation_flag", Int64, self.state_observation_flag_callback)
            rospy.Subscriber("/joint1_state", Float64, self.joint1_state_callback)
            rospy.Subscriber("/joint3_state", Float64, self.joint3_state_callback)
            rospy.Subscriber("/joint5_state", Float64, self.joint5_state_callback)
            
            pub_1 = rospy.Publisher("/action_num", Int64, queue_size = 1)
            pub_2 = rospy.Publisher("/num_state", Int64, queue_size = 1)
            pub_4 = rospy.Publisher("/num_step", Int64, queue_size = 1)
            pub_5 = rospy.Publisher("/num_episode", Int64, queue_size = 1)

            loop_rate = rospy.Rate(100)
            
            count = 0
            count_temp = 0

            self.state = self.init_state
            self.next_state = self.init_state
            self.joint1 = self.state/700
            self.joint3 = (self.state%700)/10
            self.joint5 = (self.state%10)/1
            print "joint1 : ", self.joint1
            print "joint3 : ", self.joint3
            print "joint5 : ", self.joint5
            self.next_joint1 = self.next_state/700
            self.next_joint3 = (self.next_state%700)/10
            self.next_joint5 = (self.next_state%10)/1
            print "next joint1 : ", self.next_joint1
            print "next joint3 : ", self.next_joint3
            print "next joint5 : ", self.next_joint5

            step_count = 0
            episode_count = 0
            
            episode_now = 0
            episode_past = 0

            temp_count = 0

            loss_list = []

            print "Q Learning Start!!"

            while not rospy.is_shutdown():
                if self.wait_flag:
                    print "wait 2 seconds!!"
                    count += 1
                    if count == 200:
                        self.wait_flag = False
                        self.select_action_flag = False
                        self.q_update_flag = False
                        self.state_observation_flag = True
                        self.state_observation_flag1 = True
                        self.state_observation_flag2 = True
                        self.state_observation_flag3 = True
                        count = 0
                    if count == 10:
                        self.action_num = 0
                        self.num_state = self.init_next
                        self.state = self.init_next
                        self.joint1 = self.state/700
                        self.joint3 = (self.state%700)/10
                        self.joint5 = (self.state%10)/1
                        self.reward = 0
                        pub_1.publish(self.action_num)
                        pub_2.publish(self.num_state)
                else:
                    if self.select_action_flag:
                        self.action = self.epsilon_greedy(self.joint1, self.joint3, self.joint5)
                        self.action_num = self.action
                        print "self.action_num : ", self.action_num
                        pub_1.publish(self.action_num)
                        self.num_state = self.state
                        pub_2.publish(self.num_state)
                        self.select_action_flag = False
                        print "publish /num_state"

                    if self.state_observation_flag and self.state_observation_flag1 and self.state_observation_flag2 and self.state_observation_flag3:
                        self.next_state = self.joint_to_s_client(self.joint_state[0], self.joint_state[1], self.joint_state[2])
                        print "self.joint_state[0] : ",self.joint_state[0]
                        print "self.joint_state[1] : ",self.joint_state[1]
                        print "self.joint_state[2] : ",self.joint_state[2]
                        print "s = ", self.state
                        #  self.joint1 = self.state/700
                        #  self.joint3 = (self.state%700)/10
                        #  self.joint5 = (self.state%10)/1 
                        print "now joint1 : ", self.joint1
                        print "now joint3 : ", self.joint3
                        print "now joint5 : ", self.joint5
                        print "sd = ", self.next_state
                        self.next_joint1 = self.next_state/700
                        self.next_joint3 = (self.next_state%700)/10
                        self.next_joint5 = (self.next_state%10)/1
                        print "next joint1 : ", self.next_joint1
                        print "next joint3 : ", self.next_joint3
                        print "next joint5 : ", self.next_joint5

                        self.reward = self.reward_calculation_client(step_count)
                        print "reward : ", self.reward
                        #  self.select_action_flag = True
                        self.q_update_flag = True
                        self.state_observation_flag = False
                        self.state_observation_flag1 = False
                        self.state_observation_flag2 = False
                        self.state_observation_flag3 = False

                    if self.q_update_flag:
                        #  target_val = self.reward + self.GAMMA * np.max(self.forward(self.next_joint1, self.next_joint3, self.next_joint5).data)
                        #  self.optimizer.zero_grads()
                        #  tt = xp.copy(self.q_list.data)
                        #  tt[0][self.action] = target_val
                        #  target = chainer.Variable(tt)
                        #  loss = 0.5 * (target - self.q_list) ** 2
                        #  self.ALPHA = float(self.ALPHA)
                        #  loss.grad = xp.array([[self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA, self.ALPHA]], dtype=xp.float32)
                        #  loss_list.append(np.max(loss.data))
                        #  loss.backward()
                        #  self.optimizer.update()


                        self.select_action_flag = True
                        self.q_update_flag = False
                        step_count += 1
                        print "episode : %d " % episode_count,
                        print "step : %d " % step_count,
                        print "now state : %d" % self.state,
                        print "now action : %d" % self.action,
                        #  print "loss : ", np.max(loss.data),
                        print "reward : %.0f  " % self.reward,
                        print "EPSILON : %.5f " % self.EPSILON
                        print ""

                    #  print ""

                    if self.reward >= 1:
                        print "episode : %d " % episode_count,
                        print "step : %d " % step_count,
                        print "now state : %d" % self.state,
                        print "now action : %d" % self.action,
                        #  print "loss average : %.3f " % (sum(loss_list)/len(loss_list)),
                        print "reward : %.0f  " % self.reward,
                        print "EPSILON : %.5f " % self.EPSILON,
                        print "succsess!!"
                        print ""

                        temp_result = np.array(([[episode_count, step_count]]), dtype=np.int32)
                        if episode_count == 0:
                            test_result = temp_result
                        else:
                            test_result = np.r_[test_result, temp_result]

                        
                        while 1:
                            rand_joint1 = randint(0, 21-1)-10
                            rand_joint3 = randint(0, 31-1)+40
                            rand_joint5 = randint(0, 1-1)+30
                            init_next_temp = (rand_joint1 - self.init_state_joint1) * 700 + (rand_joint3 - self.init_state_joint3) * 10 + (rand_joint5 - self.init_state_joint5) * 1
                            print "init_next_temp : ", init_next_temp
                            if (rand_joint3>=40 and rand_joint3<=47) and (rand_joint1>=-5 and rand_joint1<5):
                                print "one more!!"
                            else:
                                self.init_next = init_next_temp
                                break


                        step_count = 0
                        episode_count += 1
                        episode_now = episode_count

                        self.action_num = 0
                        self.num_state =self. init_next
                        pub_1.publish(self.action_num)
                        pub_2.publish(self.num_state)
                        loss_list = []

                        self.wait_flag = True
                    else:
                        if step_count < 100:

                            self.state = self.next_state
                            self.joint1 = self.state/700
                            self.joint3 = (self.state%700)/10
                            self.joint5 = (self.state%10)/1 

                            episode_past = episode_now
                        else:
                            print "episode : %d " % episode_count,
                            print "step : %d " % step_count,
                            print "now state : %d" % self.state,
                            print "now action : %d" % self.action,
                            #  print "loss average : %.3f " % (sum(loss_list)/len(loss_list)),
                            print "reward : %.0f  " % self.reward,
                            print "EPSILON : %.5f " % self.EPSILON,
                            print "failuer!!"
                            print ""

                            temp_result = np.array(([[episode_count, step_count]]), dtype=np.int32)
                            if episode_count == 0:
                                test_result = temp_result
                            else:
                                test_result = np.r_[test_result, temp_result]
                            

                            while 1:
                                rand_joint1 = randint(0, 21-1)-10
                                rand_joint3 = randint(0, 31-1)+40
                                rand_joint5 = randint(0, 1-1)+30
                                init_next_temp = (rand_joint1 - self.init_state_joint1) * 700 + (rand_joint3 - self.init_state_joint3) * 10 + (rand_joint5 - self.init_state_joint5) * 1
                                print "init_next_temp : ", init_next_temp
                                if (rand_joint3>=40 and rand_joint3<=47) and (rand_joint1>=-5 and rand_joint1<5):
                                    print "one more!!"
                                else:
                                    self.init_next = init_next_temp
                                    break
                            
                            step_count = 0
                            episode_count += 1
                            episode_now = episode_count

                            self.action_num = 0
                            self.num_state = self.init_next
                            pub_1.publish(self.action_num)
                            pub_2.publish(self.num_state)
                            loss_list = []

                            self.wait_flag = True


                    self.num_step = step_count
                    pub_4.publish(self.num_step)
                    self.num_episode = episode_count
                    pub_5.publish(self.num_episode)
                    


                loop_rate.sleep()
    
if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
