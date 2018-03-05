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
// int init_state_joint1 = -10;
int init_state_joint3 = 40;
int init_state_joint5 = 30;

int init_state = 3900;

float joint_state[3];
float joint_com[3];
float init_joint[3];

bool pub_flag = true;

void joint1_com_callback(std_msgs::Float64 msg){
	joint_com[0] = msg.data;
	// cout<<"joint1 command : "<<joint_com[0]<<endl;
	pub_flag = true;
}
void joint3_com_callback(std_msgs::Float64 msg){
	joint_com[1] = msg.data;
	// cout<<"joint3 command : "<<joint_com[1]<<endl;
}
void joint5_com_callback(std_msgs::Float64 msg){
	joint_com[2] = msg.data;
	// cout<<"joint5 command : "<<joint_com[2]<<endl;
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

	ros::Subscriber sub_1 =n.subscribe("/infant/joint1_position_controller/command", 10, joint1_com_callback); 
	ros::Subscriber sub_2 =n.subscribe("/infant/joint3_position_controller/command", 10, joint3_com_callback); 
	ros::Subscriber sub_3 =n.subscribe("/infant/joint5_position_controller/command", 10, joint5_com_callback); 
	ros::Subscriber sub_4 = n.subscribe("/infant/joint_states", 10, joint_states_callback);

	// ros::Publisher pub = n.advertise<std_msgs::Int64>("/select_action_flag",1);
	ros::Publisher pub = n.advertise<std_msgs::Int64>("/state_observation_flag",1);

	ros::Rate loop_rate(100);

	// std_msgs::Int64 select_action_flag;
	// select_action_flag.data = 0;

	std_msgs::Int64 state_observation_flag;
	state_observation_flag.data = 0;
	int count = 0;

	init_joint[0] = init_state/700 + init_state_joint1;
	init_joint[1] = (init_state%700)/10 + init_state_joint3;
	init_joint[2] = (init_state%10)/1 + init_state_joint5;
    
	cout<<"init joint1 : "<<init_joint[0]<<endl;
	cout<<"init joint3 : "<<init_joint[1]<<endl;
	cout<<"init joint5 : "<<init_joint[2]<<endl;
	


	joint_com[0] = init_joint[0] / 180.0 * pi;
	joint_com[1] = init_joint[1] / 180.0 * pi;
	joint_com[2] = init_joint[2] / 180.0 * pi;

	while(ros::ok()){
		// cout<<"init joint1 : "<<init_joint[0]<<endl;
		// cout<<"init joint3 : "<<init_joint[1]<<endl;
		// cout<<"init joint5 : "<<init_joint[2]<<endl;

		cout<<"join1_state : "<<joint_state[0]<<endl;
		cout<<"join1_com : "<<joint_com[0]<<endl;
		cout<<"join1_state - joint1_com = "<<fabs(joint_state[0]-joint_com[0])<<endl;
		cout<<"join3_state - joint3_com = "<<fabs(joint_state[1]-joint_com[1])<<endl;
		cout<<"join5_state - joint5_com = "<<fabs(joint_state[2]-joint_com[2])<<endl;
		if(pub_flag){
			if((fabs(joint_state[0]-joint_com[0])<=1e-3) && 
					(fabs(joint_state[1]-joint_com[1])<=1e-3) && (fabs(joint_state[2]-joint_com[2])<=1e-3)){
				// cout<<"join1_state - joint1_com = "<<fabs(joint_state[0]-joint_com[0])<<endl;
				
				// pub.publish(select_action_flag);
				pub.publish(state_observation_flag);
				cout<<"publish !"<<endl;
				// if(count>=20){
					// pub_flag = false;
				// }
				pub_flag = false;
			
			}
		}
		cout<<endl;

		count++;
		cout<<count<<endl;
		// pub.publish(select_action_flag);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
