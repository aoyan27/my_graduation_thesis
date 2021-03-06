#!/usr/bin/env python
#coding:utf-8
import argparse

import rospy

import numpy as np
import chainer
import chainer.functions as F
from chainer import cuda, optimizers

from std_msgs.msg import Float64, Int64

import math
from random import random, randint

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
	init_state_joint2 = -95
	init_state_joint3 = 150
	init_state_joint4 = -175
	init_state_joint5 = -95

        init_joint1 = 0
        init_joint2 = 95
        init_joint3 = 150
        init_joint4 = 0
        init_joint5 = 95

        init_next_joint1 = init_joint1
        init_next_joint2 = init_joint2
        init_next_joint3 = init_joint3
        init_next_joint4 = init_joint4
        init_next_joint5 = init_joint5
	
        init_state = 17500
        init_next = 17500

	num_a = 3**5

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
	state_observation_flag4 = False
	state_observation_flag5 = False

	def __init__(self):
		self.joint_state = np.zeros((5), dtype=np.float32)
		#  print self.joint_state
		self.action_num = 0
		self.reward = 0.0

		self.num_step = 0
                self.num_episode = 0


                f = open('/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model_0.dat', 'rb')
                self.model = pickle.load(f)
                if args.gpu >= 0:
                    self.model.to_gpu()
                self.optimizer = optimizers.SGD()
                self.optimizer.setup(self.model)
                self.q_list = chainer.Variable(xp.zeros((1, 243), dtype=xp.float32))

                self.action = 0
                self.joint1 = self.init_state_joint1
                self.joint2 = self.init_state_joint2
                self.joint3 = self.init_state_joint3
                self.joint4 = self.init_state_joint4
                self.joint5 = self.init_state_joint5
                self.next_joint1 = self.init_state_joint1
                self.next_joint2 = self.init_state_joint2
                self.next_joint3 = self.init_state_joint3
                self.next_joint4 = self.init_state_joint4
                self.next_joint5 = self.init_state_joint5


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

        def joint4_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[3] = msg.data
            print "joint4_state : ",self.joint_state[3]
	    self.state_observation_flag4 = True

        def joint5_state_callback(self, msg):
            #  print "msg.data : ", msg.data
            self.joint_state[4] = msg.data
            print "joint5_state : ",self.joint_state[4]
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

        def forward(self, joint1_data, joint2_data, joint3_data, joint4_data, joint5_data):
            joint1_data_float = float(joint1_data -self.mean(175.0, -175.0) ) / (175.0 - (-175.0))
            joint2_data_float = float(joint2_data -self.mean(95.0, -95.0) ) / (95.0 - (-95.0))
            joint3_data_float = float(joint3_data - self.mean(150.0, -10.0)) / (150.0 - (-10.0))
            joint4_data_float = float(joint4_data - self.mean(175.0, -175.0)) / (175.0 - (-175.0))
            joint5_data_float = float(joint5_data -self.mean(95.0, -95.0) ) / (95.0 - (-95.0))
            #  joint1_data_float = float(self.joint_state[0])
            #  joint3_data_float = float(self.joint_state[1])
            #  joint5_data_float = float(self.joint_state[2])
            #  action_data_float = float(self.action) / 27.0

            x = chainer.Variable(xp.array([[joint1_data_float, joint2_data_float, joint3_data_float, joint4_data_float, joint5_data_float]], dtype=xp.float32))
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
            #  h6 = F.relu(self.model.l6(h5))
            y = self.model.l6(h5)
            print "y : ", y.data

            return y
            	
	def select_action(self, joint1, joint2, joint3, joint4, joint5):
            self.q_list = self.forward(joint1, joint2, joint3, joint4, joint5)
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

	def epsilon_greedy(self, joint1, joint2, joint3, joint4, joint5):
	    if self.EPSILON > random():
		a = randint(0, self.num_a-1)
                print "random select!!"
	    else:
		a = self.select_action(joint1, joint2, joint3, joint4, joint5)
                print "argmax Q select!!"
	    return a
        
        def main(self):
            rospy.init_node('q_update_client_dqn')
            
            rospy.Subscriber("/state_observation_flag", Int64, self.state_observation_flag_callback)
            rospy.Subscriber("/joint1_state", Float64, self.joint1_state_callback)
            rospy.Subscriber("/joint2_state", Float64, self.joint2_state_callback)
            rospy.Subscriber("/joint3_state", Float64, self.joint3_state_callback)
            rospy.Subscriber("/joint4_state", Float64, self.joint4_state_callback)
            rospy.Subscriber("/joint5_state", Float64, self.joint5_state_callback)

            
            pub_1 = rospy.Publisher("/joint1_pose", Float64, queue_size = 1)
            pub_2 = rospy.Publisher("/joint2_pose", Float64, queue_size = 1)
            pub_3 = rospy.Publisher("/joint3_pose", Float64, queue_size = 1)
            pub_4 = rospy.Publisher("/joint4_pose", Float64, queue_size = 1)
            pub_5 = rospy.Publisher("/joint5_pose", Float64, queue_size = 1)
            pub_6 = rospy.Publisher("/action_num", Int64, queue_size = 1)
            pub_7 = rospy.Publisher("/num_step", Int64, queue_size = 1)
            pub_8 = rospy.Publisher("/num_episode", Int64, queue_size = 1)

            loop_rate = rospy.Rate(100)
            
            filename_result = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_results/test_result.txt"

            count = 0
            count_temp = 0

            self.joint1 = self.init_joint1
            self.joint2 = self.init_joint2
            self.joint3 = self.init_joint3
            self.joint4 = self.init_joint4
            self.joint5 = self.init_joint5
            print "joint1 : ", self.joint1
            print "joint2 : ", self.joint2
            print "joint3 : ", self.joint3
            print "joint4 : ", self.joint4
            print "joint5 : ", self.joint5

            self.next_joint1 = self.init_joint1
            self.next_joint2 = self.init_joint2
            self.next_joint3 = self.init_joint3
            self.next_joint4 = self.init_joint4
            self.next_joint5 = self.init_joint5
            print "next joint1 : ", self.next_joint1
            print "next joint2 : ", self.next_joint2
            print "next joint3 : ", self.next_joint3
            print "next joint4 : ", self.next_joint4
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
                        self.state_observation_flag4 = True
                        self.state_observation_flag5 = True
                        count = 0
                    if count == 10:
                        self.action_num = 0
                        self.joint1 = self.init_next_joint1
                        self.joint2 = self.init_next_joint2
                        self.joint3 = self.init_next_joint3
                        self.joint4 = self.init_next_joint4
                        self.joint5 = self.init_next_joint5
                        self.reward = 0.0
                        pub_1.publish(self.self.joint1)
                        pub_2.publish(self.self.joint2)
                        pub_3.publish(self.self.joint3)
                        pub_4.publish(self.self.joint4)
                        pub_5.publish(self.self.joint5)
                        pub_6.publish(self.action_num)
                else:
                    if self.select_action_flag:
                        self.action = self.epsilon_greedy(self.joint1, self.joint2, self.joint3, self.joint4, self.joint5)
                        self.action_num = self.action
                        print "self.action_num : ", self.action_num
                        pub_1.publish(self.joint1) 
                        pub_2.publish(self.joint2) 
                        pub_3.publish(self.joint3) 
                        pub_4.publish(self.joint4) 
                        pub_5.publish(self.joint5) 
                        pub_6.publish(self.action_num)
                        self.select_action_flag = False

                    if self.state_observation_flag and self.state_observation_flag1 and self.state_observation_flag2 and self.state_observation_flag3 and self.state_observation_flag4 and self.state_observation_flag5:
                        print "self.joint_state[0] : ",self.joint_state[0]
                        print "self.joint_state[1] : ",self.joint_state[1]
                        print "self.joint_state[2] : ",self.joint_state[2]
                        print "self.joint_state[3] : ",self.joint_state[3]
                        print "self.joint_state[4] : ",self.joint_state[4]
                        
                        print "now joint1 : ", self.joint1
                        print "now joint2 : ", self.joint2
                        print "now joint3 : ", self.joint3
                        print "now joint4 : ", self.joint4
                        print "now joint5 : ", self.joint5
                        
                        self.next_joint1 = self.joint_state[0]
                        self.next_joint2 = self.joint_state[1]
                        self.next_joint3 = self.joint_state[2]
                        self.next_joint4 = self.joint_state[3]
                        self.next_joint5 = self.joint_state[4]
                        print "next joint1 : ", self.next_joint1
                        print "next joint2 : ", self.next_joint2
                        print "next joint3 : ", self.next_joint3
                        print "next joint4 : ", self.next_joint4
                        print "next joint5 : ", self.next_joint5

                        self.reward = self.reward_calculation_client(step_count)
                        print "reward : ", self.reward
                        #  self.select_action_flag = True
                        self.q_update_flag = True
                        self.state_observation_flag = False
                        self.state_observation_flag1 = False
                        self.state_observation_flag2 = False
                        self.state_observation_flag3 = False
                        self.state_observation_flag4 = False
                        self.state_observation_flag5 = False

                    if self.q_update_flag:
                        #  target_val = self.reward + self.GAMMA * np.max(self.forward(self.next_joint1, self.next_joint2, self.next_joint3, self.next_joint4, self.next_joint5).data)
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
                        step_count += 1
                        print "episode : %d " % episode_count,
                        print "step : %d " % step_count,
                        print "now joint1 : %d " % self.joint1,
                        print "now joint2 : %d " % self.joint2,
                        print "now joint3 : %d " % self.joint3,
                        print "now joint4 : %d " % self.joint4,
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
                        print "now joint2 : %d " % self.joint2,
                        print "now joint3 : %d " % self.joint3,
                        print "now joint4 : %d " % self.joint4,
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

                        
                        while 1:
                            rand_joint1 = randint(0, 21-1)-10
                            rand_joint2 = randint(0, 21-1)-10
                            rand_joint3 = randint(0, 1-1)+40
                            rand_joint4 = randint(0, 21-1)-10
                            rand_joint5 = randint(0, 1-1)+30
                            
                            if rand_joint1 == -10:
                                print "one more!"
                            else:
                                self.init_next_joint1 = rand_joint1
                                self.init_next_joint2 = rand_joint2
                                self.init_next_joint3 = rand_joint3
                                self.init_next_joint4 = rand_joint4
                                self.init_next_joint5 = rand_joint5
                                break

                        step_count = 0
                        episode_count += 1
                        episode_now = episode_count

                        self.action_num = 0
                        self.joint1 = self.init_next_joint1
                        self.joint2 = self.init_next_joint2
                        self.joint3 = self.init_next_joint3
                        self.joint4 = self.init_next_joint4
                        self.joint5 = self.init_next_joint5
                        pub_1.publish(self.joint1)
                        pub_2.publish(self.joint2)
                        pub_3.publish(self.joint3)
                        pub_4.publish(self.joint4)
                        pub_5.publish(self.joint5)
                        pub_6.publish(self.action_num)
                        loss_list = []

                        self.wait_flag = True
                    else:
                        if step_count < 300:

                            self.joint1 = self.next_joint1
                            self.joint2 = self.next_joint2
                            self.joint3 = self.next_joint3
                            self.joint4 = self.next_joint4
                            self.joint5 = self.next_joint5

                            episode_past = episode_now
                        else:
                            print "episode : %d " % episode_count,
                            print "step : %d " % step_count,
                            print "now joint1 : %d " % self.joint1,
                            print "now joint2 : %d " % self.joint2,
                            print "now joint3 : %d " % self.joint3,
                            print "now joint4 : %d " % self.joint4,
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
                            
                            while 1:
                                rand_joint1 = randint(0, 21-1)-10
                                rand_joint2 = randint(0, 21-1)-10
                                rand_joint3 = randint(0, 1-1)+40
                                rand_joint4 = randint(0, 21-1)-10
                                rand_joint5 = randint(0, 1-1)+30
                                
                                if rand_joint1 == -10:
                                    print "one more!"
                                else:
                                    self.init_next_joint1 = rand_joint1
                                    self.init_next_joint2 = rand_joint2
                                    self.init_next_joint3 = rand_joint3
                                    self.init_next_joint4 = rand_joint4
                                    self.init_next_joint5 = rand_joint5
                                    break
                            
                            step_count = 0
                            episode_count += 1
                            episode_now = episode_count

                            self.action_num = 0
                            self.joint1 = self.init_next_joint1
                            self.joint2 = self.init_next_joint2
                            self.joint3 = self.init_next_joint3
                            self.joint4 = self.init_next_joint4
                            self.joint5 = self.init_next_joint5
                            pub_1.publish(self.joint1)
                            pub_2.publish(self.joint2)
                            pub_3.publish(self.joint3)
                            pub_4.publish(self.joint4)
                            pub_5.publish(self.joint5)
                            pub_6.publish(self.action_num)
                            loss_list = []

                            self.wait_flag = True

                    #  if math.fabs(episode_now - episode_past) > 1e-6:
                        #  if self.EPSILON > 0.3000:
                            #  self.EPSILON -= 0.0002

                    self.num_step = step_count
                    pub_7.publish(self.num_step)
                    self.num_episode = episode_count
                    pub_8.publish(self.num_episode)
                    
                    #  if episode_count%50 == 0:
                        #  model_filename = "/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model_%d.dat" % episode_count
                        #  f = open(model_filename, 'w')
                        #  pickle.dump(self.model, f)

                    #  if episode_count > 5000:
                        #  np.savetxt(filename_result, test_result, fmt="%d", delimiter=",")
                        #  f = open('/home/amsl/ros_catkin_ws/src/arm_q_learning/dqn_model/dqn_arm_model.dat', 'w')
                        #  pickle.dump(self.model, f)
                        #  print "Finish!!!"
                        #  break


                loop_rate.sleep()
    
if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
