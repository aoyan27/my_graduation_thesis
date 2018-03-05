#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <stdio.h>
using namespace std;

double pi = 3.141592;

std_msgs::Float64 joint1_pose_com;
std_msgs::Float64 joint2_pose_com;
std_msgs::Float64 joint3_pose_com;
std_msgs::Float64 joint5_pose_com;

std_msgs::Float64 state_observation_flag;
int joint_int[3];
int num_episode;
int num_step;
bool pub1_flag = false;
bool pub2_flag = false;
bool pub3_flag = false;


void num_step_callback(std_msgs::Int64 msg){
	num_step = msg.data;
}

void num_episode_callback(std_msgs::Int64 msg){
	num_episode = msg.data;
}

void joint1_callback(std_msgs::Float64 msg){
	joint1_pose_com.data = msg.data/180.0 * pi;
	joint_int[0] = msg.data;
	cout<<"joint1 : "<<joint_int[0]<<endl;
	pub1_flag = true;
}

void joint2_callback(std_msgs::Float64 msg){
	joint2_pose_com.data = msg.data/180.0 * pi ;
	joint_int[1] = msg.data;
	cout<<"joint2 : "<<joint_int[1]<<endl;
	pub2_flag = true;
}

void joint3_callback(std_msgs::Float64 msg){
	joint3_pose_com.data = msg.data/180.0 * pi;
	cout<<"joint3(msg) : "<<msg.data<<endl;;
	joint_int[2] = msg.data;
	cout<<"joint3 : "<<joint_int[2]<<endl;
	pub3_flag = true;
}

void joint5_callback(std_msgs::Float64 msg){
	joint5_pose_com.data = msg.data/180.0 * pi;
	cout<<"joint5(msg) : "<<msg.data<<endl;;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "arm_control_tis");
	ros::NodeHandle n;

	ros::Subscriber sub_1 = n.subscribe("/joint1_pose_com", 10, joint1_callback);
	ros::Subscriber sub_2 = n.subscribe("/joint2_pose_com", 10, joint2_callback);
	ros::Subscriber sub_3 = n.subscribe("/joint3_pose_com", 10, joint3_callback);
	ros::Subscriber sub_5 = n.subscribe("/joint5_pose_com", 10, joint5_callback);

	ros::Subscriber sub_state = n.subscribe("/num_episode", 10, num_episode_callback);
	ros::Subscriber sub_step = n.subscribe("/num_step", 10, num_step_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/infant/joint1_position_controller/command", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/infant/joint2_position_controller/command", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/infant/joint3_position_controller/command", 1);
	ros::Publisher pub_5 = n.advertise<std_msgs::Float64>("/infant/joint5_position_controller/command", 1);
	// ros::Publisher pub_4 = n.advertise<std_msgs::Float64>("/state_observation_flag", 1);


	ros::Rate loop_rate(100);

	state_observation_flag.data = 0.0;

	while(ros::ok()){
		if(pub1_flag && pub2_flag && pub3_flag){

			printf("num episode : %d\n", num_episode);
			printf("num step : %d\n", num_step);
			


			pub_1.publish(joint1_pose_com);
			pub_2.publish(joint2_pose_com);
			pub_3.publish(joint3_pose_com);
			pub_5.publish(joint5_pose_com);

			ROS_INFO("publish joint1:%lf, joint2:%lf, joint3:%lf", joint1_pose_com.data, joint2_pose_com.data, joint3_pose_com.data);
			printf("\n");

			pub1_flag = false;
			pub2_flag = false;
			pub3_flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
