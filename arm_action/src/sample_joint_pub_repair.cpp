#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;

double pi = 3.141592;

double joint1 = 0.0;
double joint2 = 95.0;
double joint3 = 150.0;
double joint4 = 0.0;
double joint5 = 95.0;

std_msgs::Float64 joint1_pose;
std_msgs::Float64 joint2_pose;
std_msgs::Float64 joint3_pose;
std_msgs::Float64 joint4_pose;
std_msgs::Float64 joint5_pose;

bool pub_flag1 = false;
bool pub_flag2 = false;
bool pub_flag3 = false;
bool pub_flag4 = false;
bool pub_flag5 = false;

void joint1_state_callback(std_msgs::Float64 msg){
	joint1_pose.data = msg.data;
	pub_flag1 = true;
}

void joint2_state_callback(std_msgs::Float64 msg){
	joint2_pose.data = msg.data;
	pub_flag2 = true;
}

void joint3_state_callback(std_msgs::Float64 msg){
	joint3_pose.data = msg.data;
	pub_flag3 = true;
}

void joint4_state_callback(std_msgs::Float64 msg){
	joint4_pose.data = msg.data;
	pub_flag4 = true;
}

void joint5_state_callback(std_msgs::Float64 msg){
	joint5_pose.data = msg.data;
	pub_flag5 = true;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "sample_joint_pub");
	ros::NodeHandle n;

	ros::Subscriber sub_1 = n.subscribe<std_msgs::Float64>("/joint1_state", 10, joint1_state_callback);
	ros::Subscriber sub_2 = n.subscribe<std_msgs::Float64>("/joint2_state", 10, joint2_state_callback);
	ros::Subscriber sub_3 = n.subscribe<std_msgs::Float64>("/joint3_state", 10, joint3_state_callback);
	ros::Subscriber sub_4 = n.subscribe<std_msgs::Float64>("/joint4_state", 10, joint4_state_callback);
	ros::Subscriber sub_5 = n.subscribe<std_msgs::Float64>("/joint5_state", 10, joint5_state_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/action_type_joint1", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/action_type_joint2", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/action_type_joint3", 1);
	ros::Publisher pub_4 = n.advertise<std_msgs::Float64>("/action_type_joint4", 1);
	ros::Publisher pub_5 = n.advertise<std_msgs::Float64>("/action_type_joint5", 1);
	ros::Publisher pub_6 = n.advertise<std_msgs::Float64>("/joint1_pose", 1);
	ros::Publisher pub_7 = n.advertise<std_msgs::Float64>("/joint2_pose", 1);
	ros::Publisher pub_8 = n.advertise<std_msgs::Float64>("/joint3_pose", 1);
	ros::Publisher pub_9 = n.advertise<std_msgs::Float64>("/joint4_pose", 1);
	ros::Publisher pub_10 = n.advertise<std_msgs::Float64>("/joint5_pose", 1);

	ros::Rate loop_rate(10);

	std_msgs::Float64 action_type_joint1;
	std_msgs::Float64 action_type_joint2;
	std_msgs::Float64 action_type_joint3;
	std_msgs::Float64 action_type_joint4;
	std_msgs::Float64 action_type_joint5;


	joint1_pose.data = joint1;
	joint2_pose.data = joint2;
	joint3_pose.data = joint3;
	joint4_pose.data = joint4;
	joint5_pose.data = joint5;
	while(ros::ok()){
		if(pub_flag1 && pub_flag2 && pub_flag3 && pub_flag4 && pub_flag5){
			action_type_joint1.data = 1.0;
			// joint1_pose.data = joint1;
			// pub_1.publish(action_type_joint1);
			pub_6.publish(joint1_pose);

			action_type_joint2.data = 2.0;
			// joint2_pose.data = joint2;
			// pub_2.publish(action_type_joint2);
			pub_7.publish(joint2_pose);
			
			action_type_joint3.data = 1.0;
			// joint3_pose.data = joint3;
			// pub_3.publish(action_type_joint3);
			pub_8.publish(joint3_pose);

			action_type_joint4.data = 1.0;
			// joint4_pose.data = joint4;
			// pub_4.publish(action_type_joint4);
			pub_9.publish(joint4_pose);
			
			action_type_joint5.data = 2.0;
			// joint5_pose.data = joint5;
			// pub_5.publish(action_type_joint5);
			pub_10.publish(joint5_pose);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
