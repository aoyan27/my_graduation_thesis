#!/usr/bin/env python
#coding:utf-8
import rospy
import numpy as np
from std_msgs.msg import Float64, Int64
from sensor_msgs.msg import JointState
import math
from random import random, randint

from joint_to_s.srv import *
from reward_calculation.srv import *

class agent:
	pi = 3.141592
	init_state_joint1 = -35
	init_state_joint3 = 40
	init_state_joint5 = 30

	init_state = 3900

	num_s = 70*70*10
	num_a = 3*3*3

	ALPHA = 0.5
	GAMMA = 0.9
        #  EPSILON = 0.999
        EPSILON = 0.0

	wait_flag = False
	select_action_flag = False
	q_update_flag = False
	state_observation_flag = False

	def __init__(self):
		self.joint_state = np.zeros((3), dtype=np.float32)
		#  print self.joint_state
		self.action_num = 0
		self.reward = 0.0

		self.num_state = 0
		self.num_step = 0

		self.state = 0
		self.next_state = 0
		self.Qtable = np.loadtxt("/home/amsl/ros_catkin_ws/src/arm_q_learning/q_table/q_table_cari9/q_table_1500.txt", delimiter=",")
                print self.Qtable[3900]
                self.Qmax = 0.0

                self.action = 0
                self.state = 0
                self.next_state = 0

                self.state_list = []

	def state_observation_flag_callback(self, msg):
		self.state_observation_flag = True
		#  print "flag !!!"

	def joint_states_callback(self, msg):
		self.joint_state[0] = msg.position[0] / self.pi * 180
		self.joint_state[1] = msg.position[2] / self.pi * 180
		self.joint_state[2] = msg.position[4] / self.pi * 180
                #  print "joint1 : ", self.joint_state[0]
                #  print "joint3 : ", self.joint_state[1]
                #  print "joint5 : ", self.joint_state[2]
        
        def joint_to_s_client(self):
            #  rospy.wait_for_service('state_num')
            req_joint1 = self.joint_state[0]
            req_joint3 = self.joint_state[1]
            req_joint5 = self.joint_state[2]
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
            



        def max_Qval(self, s):
		max_temp = self.Qtable[s][0]
		for i in range(1, self.num_a):
			if self.Qtable[s][i] > max_temp:
				max_temp = self.Qtable[s][i]
		return max_temp
	
	def select_action(self, s):
		i_max = []
		i_max.append(0)
		max_temp = self.Qtable[s][0]
		for i in range(1, self.num_a):
			if self.Qtable[s][i]>max_temp:
				max_temp = self.Qtable[s][i]
				i_max = []
				i_max.append(i)
			elif self.Qtable[s][i] == max_temp:
				i_max.append(i)
		a = i_max[randint(0, len(i_max)-1)]
		return a

	def epsilon_greedy(self, s):
		if self.EPSILON > random():
			a = randint(0, self.num_a-1)
		else:
			a = self.select_action(s)
		return a        
        
        
        def main(self):
            rospy.init_node('q_update_demo_python')
            
            rospy.Subscriber("/state_observation_flag", Int64, self.state_observation_flag_callback)
            rospy.Subscriber("/infant/joint_states", JointState, self.joint_states_callback)
            
            pub_1 = rospy.Publisher("/action_num", Int64, queue_size = 1)
            pub_2 = rospy.Publisher("/num_state", Int64, queue_size = 1)
            pub_4 = rospy.Publisher("/num_step", Int64, queue_size = 1)
            
            pub = rospy.Publisher("/test", Int64, queue_size = 1)


            loop_rate = rospy.Rate(100)
            
            count = 0
            count_temp = 0

            self.state = self.init_state
            self.next_state = self.init_state

            step_count = 0
            episode_count = 0
            
            episode_now = 0
            episode_past = 0
            
            while not rospy.is_shutdown():
                pub.publish(episode_count)
                if self.wait_flag:
                    #  print "wait 4 seconds!!"
                    print "count : ",count
                    if count == 0:
                        print "state_list(before) : ", self.state_list
                        self.state_list.reverse()
                        print "state_list(after) : ", self.state_list
                    if count%2 == 0:
                        self.action_num = 0
                        #  self.num_state = self.init_state
                        if count/10 < len(self.state_list):
                            self.num_state = self.state_list[count/10]
                        else:
                            self.num_state = self.init_state

                        self.state = self.init_state
                        self.reward = 0
                        pub_1.publish(self.action_num)
                        pub_2.publish(self.num_state)
                    
                    count += 1
                    
                    if count == 400:
                        self.wait_flag = False
                        self.select_action_flag = False
                        self.q_update_flag = False
                        self.state_observation_flag = True
                        self.state_list = []
                        count = 0
                else:
                    if self.select_action_flag:
                        print "episode : %d " % episode_count,
                        print "step : %d " % step_count,
                        print "Q[%d][%d] = %.3f " % (self.state, self.action, self.Qtable[self.state][self.action]),
                        print "TD error : %.3f " % (float(self.reward) + self.GAMMA * self.Qmax - self.Qtable[self.state][self.action]),
                        print "reward : %.0f  " % self.reward,
                        print "EPSILON : %.3f " % self.EPSILON,
                        print ""
                        self.action = self.epsilon_greedy(self.state)
                        self.action_num = self.action
                        pub_1.publish(self.action_num)
                        self.num_state = self.state
                        pub_2.publish(self.num_state)
                        self.select_action_flag = False
                        print "publish /num_state"

                    if self.state_observation_flag:
                        self.next_state = self.joint_to_s_client()
                        print "s = ", self.state
                        self.state_list.append(self.state)
                        print "sd = ", self.next_state

                        self.reward = self.reward_calculation_client(step_count)
                        print "reward : ", self.reward
                        #  self.select_action_flag = True
                        self.q_update_flag = True
                        self.state_observation_flag = False

                    if self.q_update_flag:
                        self.Qmax = self.max_Qval(self.next_state)
                        #  self.Qtable[self.state][self.action] = (1 - self.ALPHA) * self.Qtable[self.state][self.action] + self.ALPHA * (float(self.reward) + self.GAMMA *self.Qmax)
                        self.select_action_flag = True
                        self.q_update_flag = False
                        step_count += 1

                    #  print ""

                    if self.reward >= 6:
                        print "episode : %d " % episode_count,
                        print "step : %d " % step_count,
                        print "Q[%d][%d] = %.3f " % (self.state, self.action, self.Qtable[self.state][self.action]),
                        print "TD error : %.3f " % (float(self.reward) + self.GAMMA * self.Qmax - self.Qtable[self.state][self.action]),
                        print "reward : %.0f  " % self.reward,
                        print "EPSILON : %.3f " % self.EPSILON,
                        print "succsess!!"

                        step_count = 0
                        episode_count += 1
                        episode_now = episode_count

                        self.action_num = 0
                        self.num_state = self.state
                        pub_1.publish(self.action_num)
                        pub_2.publish(self.num_state)

                        self.wait_flag = True
                    else:
                        if step_count < 500:
                            #  print "episode : %d " % episode_count,
                            #  print "step : %d " % step_count,
                            #  print "Q[%d][%d] = %.3f " % (self.state, self.action, self.Qtable[self.state][self.action]),
                            #  print "TD error : %.3f " % (float(self.reward) + self.GAMMA * self.Qmax - self.Qtable[self.state][self.action]),
                            #  print "reward : %.0f  " % self.reward,
                            #  print "EPSILON : %.3f " % self.EPSILON,
                            #  print ""

                            self.state = self.next_state

                            episode_past = episode_now
                        else:
                            print "episode : %d " % episode_count,
                            print "step : %d " % step_count,
                            print "Q[%d][%d] = %.3f " % (self.state, self.action, self.Qtable[self.state][self.action]),
                            print "TD error : %.3f " % (float(self.reward) + self.GAMMA * self.Qmax - self.Qtable[self.state][self.action]),
                            print "reward : %.0f  " % self.reward,
                            print "EPSILON : %.3f " % self.EPSILON,
                            print "failuer!!"

                            step_count = 0
                            episode_count += 1
                            episode_now = episode_count

                            self.action_num = 0
                            self.num_state = self.state
                            pub_1.publish(self.action_num)
                            pub_2.publish(self.num_state)

                            self.wait_flag = True

                    #  if math.fabs(episode_now - episode_past) > 1e-6:
                        #  print ""

                    self.num_step = step_count
                    pub_4.publish(self.num_step)


                #  print "state : ", self.joint_to_s_client()
                #  print "reward : ", self.reward_calculation_client(0)
                #  rospy.spin()
                loop_rate.sleep()
    
if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
