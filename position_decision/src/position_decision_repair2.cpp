#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <stdio.h>

#include <joint_to_s/joint_state.h>

using namespace std;

double pi = 3.141592;

int init_joint1 = 0;
int init_joint3 = 65;
int init_joint5 = 45;

float joint_state[3];
float joint_com[3];
float init_joint[3];

bool pub_flag1 = true;
bool pub_flag3 = true;
bool pub_flag5 = true;

int num_step;
int num_episode;

void num_step_callback(std_msgs::Int64 msg){
	num_step = msg.data;
}

void num_episode_callback(std_msgs::Int64 msg){
	num_episode = msg.data;
}

void joint1_com_callback(std_msgs::Float64 msg){
	joint_com[0] = msg.data;
	// cout<<"joint1 command : "<<joint_com[0]<<endl;
	pub_flag1 = true;
}
void joint3_com_callback(std_msgs::Float64 msg){
	joint_com[1] = msg.data;
	// cout<<"joint3 command : "<<joint_com[1]<<endl;
	pub_flag3 = true;
}
void joint5_com_callback(std_msgs::Float64 msg){
	joint_com[2] = msg.data;
	// cout<<"joint5 command : "<<joint_com[2]<<endl;
	pub_flag5 = true;
}

void joint_states_callback(sensor_msgs::JointState msg){
	joint_state[0] = msg.position[0];
	joint_state[1] = msg.position[2];
	joint_state[2] = msg.position[4];

	// cout<<"joint1 : "<<joint_state[0]<<endl;
	// cout<<"joint3 : "<<joint_state[1]<<endl;
	// cout<<"joint5 : "<<joint_state[2]<<endl;	
}



int main(int argc, char *argv[]){
	ros::init(argc, argv, "position_dicision");
	ros::NodeHandle n;

	ros::Subscriber sub_episode = n.subscribe("/num_episode", 10, num_episode_callback);
	ros::Subscriber sub_step = n.subscribe("/num_step", 10, num_step_callback);


	ros::Subscriber sub_1 =n.subscribe("/infant/joint1_position_controller/command", 10, joint1_com_callback); 
	ros::Subscriber sub_3 =n.subscribe("/infant/joint3_position_controller/command", 10, joint3_com_callback); 
	ros::Subscriber sub_5 =n.subscribe("/infant/joint5_position_controller/command", 10, joint5_com_callback); 
	
	ros::Subscriber sub = n.subscribe("/infant/joint_states", 10, joint_states_callback);

	// ros::Publisher pub = n.advertise<std_msgs::Int64>("/select_action_flag",1);
	ros::Publisher pub = n.advertise<std_msgs::Int64>("/state_observation_flag",1);

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/joint1_state", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/joint3_state", 1);
	ros::Publisher pub_5 = n.advertise<std_msgs::Float64>("/joint5_state", 1);

	ros::Rate loop_rate(100);

	// std_msgs::Int64 select_action_flag;
	// select_action_flag.data = 0;

	std_msgs::Int64 state_observation_flag;
	state_observation_flag.data = 0;
	int count = 0;

	init_joint[0] = (float)init_joint1;
	init_joint[1] = (float)init_joint3;
	init_joint[2] = (float)init_joint5;
    
	// cout<<"init joint1 : "<<init_joint[0]<<endl;
	// cout<<"init joint3 : "<<init_joint[1]<<endl;
	// cout<<"init joint5 : "<<init_joint[2]<<endl;

	joint_com[0] = init_joint[0] / 180.0 * pi;
	joint_com[1] = init_joint[1] / 180.0 * pi;
	joint_com[2] = init_joint[2] / 180.0 * pi;

	std_msgs::Float64 joint1_state;
	std_msgs::Float64 joint3_state;
	std_msgs::Float64 joint5_state;


	while(ros::ok()){
		// cout<<"init joint1 : "<<init_joint[0]<<endl;
		// cout<<"init joint3 : "<<init_joint[1]<<endl;
		// cout<<"init joint5 : "<<init_joint[2]<<endl;

		// cout<<"join1_state : "<<joint_state[0]<<endl;
		// cout<<"join1_com : "<<joint_com[0]<<endl;
		// cout<<"join1_state - joint1_com = "<<fabs(joint_state[0]-joint_com[0])<<endl;
		// cout<<"join3_state - joint3_com = "<<fabs(joint_state[1]-joint_com[1])<<endl;
		// cout<<"join5_state - joint5_com = "<<fabs(joint_state[2]-joint_com[2])<<endl;
		if(pub_flag1 && pub_flag3 && pub_flag5){
			cout<<"num_episode : "<<num_episode<<endl;
			cout<<"num_step : "<<num_step<<endl;
			
			if((fabs(joint_state[0]-joint_com[0])<=1e-3) && (fabs(joint_state[1]-joint_com[1])<=1e-3) 
					&& (fabs(joint_state[2]-joint_com[2])<=1e-3)){
				cout<<"num_episode : "<<num_episode<<endl;
				cout<<"num_step : "<<num_step<<endl;
				// cout<<"join1_state - joint1_com = "<<fabs(joint_state[0]-joint_com[0])<<endl;
				cout<<"joint1_state : "<<joint_state[0]<<endl;
				cout<<"joint3_state : "<<joint_state[1]<<endl;
				cout<<"joint5_state : "<<joint_state[2]<<endl;

		
				if(joint_state[0]<0.0){
					joint1_state.data = (double)(int)(joint_state[0]/pi * 180.0 - 0.5);
				}
				else{
					joint1_state.data = (double)(int)(joint_state[0]/pi * 180.0 + 0.5);
				}
				if(joint_state[1]<0.0){
					joint3_state.data = (double)(int)(joint_state[1]/pi * 180.0 - 0.5);
				}
				else{
					joint3_state.data = (double)(int)(joint_state[1]/pi * 180.0 + 0.5);
				}
				if(joint_state[2]<0.0){
					joint5_state.data = (double)(int)(joint_state[2]/pi * 180.0 - 0.5);
				}
				else{
					joint5_state.data = (double)(int)(joint_state[2]/pi * 180.0 + 0.5);
				}
				
				cout<<"joint1_state(float64_msg) : "<<joint1_state<<endl;
				cout<<"joint3_state(float64_msg) : "<<joint3_state<<endl;
				cout<<"joint5_state(float64_msg) : "<<joint5_state<<endl;
				
				// pub.publish(select_action_flag);
				pub.publish(state_observation_flag);
				pub_1.publish(joint1_state);
				pub_3.publish(joint3_state);
				pub_5.publish(joint5_state);
				cout<<"publish !"<<endl;
				// if(count>=20){
					// pub_flag = false;
				// }
				pub_flag1 = false;
				pub_flag3 = false;
				pub_flag5 = false;
			
			}
		}
		// cout<<endl;

		count++;
		// cout<<cont<<endl;
		// pub.publish(select_action_flag);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
