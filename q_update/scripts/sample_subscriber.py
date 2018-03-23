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
    def __init__(self):
        self.data_size = 5
        #  self.data_size = 10**5
        #history memory : D = [s(joint1, joint2, joint3, x, y, z), a, r, s_dash(joint1, joint2, joint3, x, y, z), end_episode_flag]
        self.D = [xp.zeros((self.data_size, 1, 6), dtype=xp.float32),
                  xp.zeros(self.data_size, dtype=xp.uint8),
                  xp.zeros((self.data_size, 1), dtype=xp.float32),
                  xp.zeros((self.data_size, 1, 6), dtype=xp.float32),
                  xp.zeros((self.data_size, 1), dtype=np.bool)]
        
        self.time = 0
        self.state = xp.zeros((1, 6), dtype=xp.float32)
        self.action = 0
        self.reward = 0.0
        self.next_state = xp.zeros((1, 6), dtype=xp.float32)
        self.episode_end_flag = False

        self.time_sub_flag = False
        self.state_sub_flag = False
        self.action_sub_flag = False
        self.reward_sub_flag = False
        self.next_state_sub_flag = False
        self.episode_end_flag_sub_flag = False

    def time_callback(self, msg):
        self.time = msg.data
        self.time_sub_flag = True
        #  print "time : ", msg.data

    def state_callback(self, msg):
        self.state = msg.data
        self.state = cuda.to_gpu(self.state)
        self.state_sub_flag = True
        #  print "state : ", msg.data
    
    def action_callback(self, msg):
        self.action = msg.data
        self.action_sub_flag = True
        #  print "action : ", msg.data
    
    def reward_callback(self, msg):
        self.reward = msg.data
        self.reward_sub_flag = True
        #  print "reward : ", msg.data
    
    def next_state_callback(self, msg):
        self.next_state = msg.data
        self.next_state = cuda.to_gpu(self.next_state)
        self.next_state_sub_flag = True
        #  print "next_state : ", msg.data

    def episode_end_flag_callback(self, msg):
        self.episode_end_flag = msg.data
        self.episode_end_flag_sub_flag = True
        #  print "episode_end_flag : ", msg.data

    
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
    
    def main(self):
        rospy.init_node('sample_subscriber')
        
        rospy.Subscriber("/time", Int64, self.time_callback)
        rospy.Subscriber("/state", numpy_msg(State), self.state_callback)
        rospy.Subscriber("/action", UInt8, self.action_callback)
        rospy.Subscriber("/reward", Float32, self.reward_callback)
        rospy.Subscriber("/next_state", numpy_msg(State), self.next_state_callback)
        rospy.Subscriber("/episode_end_flag", Bool, self.episode_end_flag_callback)

        loop_rate = rospy.Rate(100)


        while not rospy.is_shutdown():
            if self.time_sub_flag and self.state_sub_flag and self.action_sub_flag \
                    and self.reward_sub_flag and self.next_state_sub_flag and self.episode_end_flag_sub_flag:
                print "state : ",self.state
                print "action : ", self.action
                print "reward : ", self.reward
                print "next_state : ",self.next_state
                print "episode_end_flag : ", self.episode_end_flag
                #  print "now loading!!"
                self.time_sub_flag = False
                self.state_sub_flag = False
                self.action_sub_flag = False
                self.reward_sub_flag = False
                self.next_state_sub_flag = False
                self.episode_end_flag_sub_flag = False
                self.stock_experience(self.time, self.state, self.action,\
                        self.reward, self.next_state, self.episode_end_flag)
                print "self.D : "
                print self.D
                
            loop_rate.sleep()



if __name__=="__main__":
    ql_agent = agent()
    ql_agent.main()
