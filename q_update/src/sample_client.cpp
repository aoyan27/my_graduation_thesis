#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <stdio.h>

#include <joint_to_s/joint_state.h>

using namespace std;

double pi = 3.141592;

int init_state_joint1 = -35;
int init_state_joint3 = 40;
int init_state_joint5 = 30;


float joint_state[3];


void joint_states_callback(sensor_msgs::JointState msg){
	joint_state[0] = msg.position[0] / pi * 180;
	joint_state[1] = msg.position[2] / pi * 180;
	joint_state[2] = msg.position[4] / pi * 180;

	// cout<<"joint1 : "<<joint_state[0]<<endl;
	// cout<<"joint3 : "<<joint_state[1]<<endl;
	// cout<<"joint5 : "<<joint_state[2]<<endl;	
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "joint_to_s");
	ros::NodeHandle n;
	
	ros::Subscriber sub_1 = n.subscribe("/infant/joint_states", 10, joint_states_callback);
	
	ros::ServiceClient client = n.serviceClient<joint_to_s::joint_state>("state_num");

	ros::Rate loop_rate(5);

	joint_to_s::joint_state srv;
	
	int i;
	int count=0;
	int state[3];

	int joint_state_int[3];

	while(ros::ok()){
		srv.request.joint1 = joint_state[0];
		srv.request.joint3 = joint_state[1];
		srv.request.joint5 = joint_state[2];
		
		if(count ==15){
			if(client.call(srv)){
				ROS_INFO("state_num: %ld", srv.response.state);
				printf("\n");
			}
			else{
				ROS_ERROR("Failed to call service joint_to_s_server");
				return 1;
			}
			count =0;
		}

		count++;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
