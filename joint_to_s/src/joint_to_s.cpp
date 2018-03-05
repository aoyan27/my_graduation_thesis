#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

using namespace std;

double pi = 3.141592;

int init_state_joint1 = -35;
int init_state_joint3 = 40;
int init_state_joint5 = 30;

// int num_s = 70*110*90;

float joint_state[3];

bool pub_flag = false;

void request_state_num_callback(std_msgs::Int64 msg){
	pub_flag = true;
	cout<<"subscribe request_state_num"<<endl;
}

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
	ros::Subscriber sub_2 = n.subscribe("/request_state_num", 10, request_state_num_callback);
	
	ros::Publisher pub = n.advertise<std_msgs::Int64>("/state_num",1);

	ros::Rate loop_rate(5);

	std_msgs::Int64 state_num;
	
	int i;
	int state[3];

	int joint_state_int[3];

	while(ros::ok()){
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
		
		state_num.data = 700 * state[0] + 10 * state[1] + 1 * state[2];

		if(state_num.data < 0){
			state_num.data = 0;
		}
		if(state_num.data > 49000){
			state_num.data = 49000;
		}

		cout<<"state number : "<<state_num.data<<endl;
		if(pub_flag){
			pub.publish(state_num);
			pub_flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
