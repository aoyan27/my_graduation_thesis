#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

using namespace std;

int state_num;

int init_state_joint1 = -35;
int init_state_joint3 = 40;
int init_state_joint5 = 30;


void state_num_callback(std_msgs::Int64 msg){
	state_num = msg.data;
	cout<<"state_num : "<<state_num<<endl;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "s_to_joint");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/state_num", 10, state_num_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/joint1_pose", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/joint3_pose", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/joint5_pose", 1);

	ros::Rate loop_rate(5);

	int joint1_pose;
	int joint3_pose;
	int joint5_pose;

	std_msgs::Float64 joint1_pose_pub;
	std_msgs::Float64 joint3_pose_pub;
	std_msgs::Float64 joint5_pose_pub;

	while(ros::ok()){
		joint1_pose = state_num/700 + init_state_joint1;
		joint3_pose = (state_num%700)/10 + init_state_joint3;
		joint5_pose = (state_num%10)/1 + init_state_joint5;

		// cout<<"joint1 pose : "<<joint1_pose<<endl;
		// cout<<"joint3 pose : "<<joint3_pose<<endl;
		// cout<<"joint5 pose : "<<joint5_pose<<endl;

		joint1_pose_pub.data = (float)joint1_pose;
		joint3_pose_pub.data = (float)joint3_pose;
		joint5_pose_pub.data = (float)joint5_pose;

		cout<<"joint1 pose pub : "<<joint1_pose_pub.data<<endl;
		cout<<"joint3 pose pub : "<<joint3_pose_pub.data<<endl;
		cout<<"joint5 pose pub : "<<joint5_pose_pub.data<<endl;

		cout<<endl;
		
		pub_1.publish(joint1_pose_pub);
		pub_2.publish(joint3_pose_pub);
		pub_3.publish(joint5_pose_pub);
		
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
