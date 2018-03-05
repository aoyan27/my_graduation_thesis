#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;

double pi = 3.141592;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "sample_joint_pub");
	ros::NodeHandle n;


	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/action_type_joint1", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/action_type_joint3", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/action_type_joint5", 1);
	ros::Publisher pub_4 = n.advertise<std_msgs::Float64>("/joint1_pose", 1);
	ros::Publisher pub_5 = n.advertise<std_msgs::Float64>("/joint3_pose", 1);
	ros::Publisher pub_6 = n.advertise<std_msgs::Float64>("/joint5_pose", 1);

	ros::Rate loop_rate(5);

	std_msgs::Float64 action_type_joint1;
	std_msgs::Float64 action_type_joint3;
	std_msgs::Float64 action_type_joint5;

	std_msgs::Float64 joint1_pose;
	std_msgs::Float64 joint3_pose;
	std_msgs::Float64 joint5_pose;

	while(ros::ok()){
		// action_type_joint1.data = 0.0;
		joint1_pose.data = 0.0;
		pub_1.publish(action_type_joint1);
		pub_4.publish(joint1_pose);

		// action_type_joint3.data = 1.0;
		joint3_pose.data =-50.0;
		pub_2.publish(action_type_joint3);
		pub_5.publish(joint3_pose);

		// action_type_joint5.data = 2.0;
		joint5_pose.data = 50.0;
		pub_3.publish(action_type_joint5);
		pub_6.publish(joint5_pose);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
