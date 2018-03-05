#include <ros/ros.h>
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

bool state_num(joint_to_s::joint_state::Request &req,
			   joint_to_s::joint_state::Response &res){
	float joint_state[3];
	joint_state[0] = req.joint1;
	joint_state[1] = req.joint3;
	joint_state[2] = req.joint5;

	int joint_state_int[3];

	int state[3];

	int i;

	for(i=0;i<3;i++){
		if(joint_state[i] > 0.0){
			joint_state_int[i] = (int) floor(joint_state[i] + 0.5);
		}
		else{
			joint_state_int[i] = -1 * (int) floor(fabs(joint_state[i]) + 0.5);
		}
	}

	if(joint_state_int[0]<-35){
		joint_state_int[0] = -35;
	}
	else if((int)joint_state[0]>34){
		joint_state_int[0] = 34;
	}

	if((int)joint_state[1]<40){
		joint_state_int[1] = 40;
	}
	else if((int)joint_state[1]>109){
		joint_state_int[1] = 109;
	}
	
	if((int)joint_state[2]<30){
		joint_state_int[2] = 30;
	}
	else if((int)joint_state[2]>39){
		joint_state_int[2] = 39;
	}

	state[0] = joint_state_int[0] - init_state_joint1;
	state[1] = joint_state_int[1] - init_state_joint3;
	state[2] = joint_state_int[2] - init_state_joint5;
	// cout<<"joint1(int)"<<state[0]<<endl;
	// cout<<"joint3(int)"<<state[1]<<endl;
	// cout<<"joint5(int)"<<state[2]<<endl;
	
	res.state = 700 * state[0] + 10 * state[1] + 1 * state[2];
	
	if(res.state < 0){
		res.state = 0;
	}
	if(res.state > 49000){
		res.state = 49000;
	}

	ROS_INFO("request: joint1=%lf, joint3=%lf, joint5=%lf", req.joint1, req.joint3, req.joint5);
	ROS_INFO("sending back response: [%ld]", res.state);
	printf("\n");
	return true;
}


int main(int argc, char *argv[]){
	ros::init(argc, argv, "joint_to_s_server");
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("state_num", state_num);
	ROS_INFO("joint_to_s_server start.");
	ros::spin();

	return 0;
}
