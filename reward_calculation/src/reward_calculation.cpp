#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <gazebo_msgs/GetLinkState.h>
#include <math.h>
#include <stdio.h>
using namespace std;

double pi = 3.141592;

bool pub_flag = false;

void request_reward_callback(std_msgs::Int64 msg){
	pub_flag = true;
	printf("acception of request reward!!\n");

}

double cal_radius(double y, double z){
	double dist_r;
	dist_r = sqrt(pow(y, 2) + pow(z, 2));

	return dist_r;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "reward_calculation");
	ros::NodeHandle n;
	ros::ServiceClient client;

	ros::Subscriber sub = n.subscribe("/request_reward", 10, request_reward_callback);
	ros::Publisher pub = n.advertise<std_msgs::Float64>("/reward", 1);

	ros::Rate loop_rate(10);

	string linkName = "infant::link8";
	std_msgs::Float64 reward;
	gazebo_msgs::GetLinkState getlinkstate;
	
	double polar_pose[3];
	double arm_pose[3];
	
	double target_pose[3];
	target_pose[0] = 0.75;
	target_pose[1] = 0.00;
	target_pose[2] = 0.83;


	while(ros::ok()){
		client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
		getlinkstate.request.link_name = linkName;	
		client.call(getlinkstate);
		arm_pose[0] = getlinkstate.response.link_state.pose.position.x;
		arm_pose[1] = getlinkstate.response.link_state.pose.position.y;
		arm_pose[2] = getlinkstate.response.link_state.pose.position.z;
		// cout<<pose[0]<<endl;
		cout<<"x : "<<getlinkstate.response.link_state.pose.position.x<<endl;
		cout<<"y : "<<getlinkstate.response.link_state.pose.position.y<<endl;
		cout<<"z : "<<getlinkstate.response.link_state.pose.position.z<<endl;

		polar_pose[0] = arm_pose[0] - target_pose[0];
		polar_pose[1] = arm_pose[1] - target_pose[1];
		polar_pose[2] = arm_pose[2] - target_pose[2];

		cout<<"r : "<<cal_radius(polar_pose[1], polar_pose[2])<<endl;;

		if(arm_pose[0] >= 0.91){
			cout<<"you can touch wall!!"<<endl;
			if(cal_radius(polar_pose[1], polar_pose[2])<=0.3 
					&& cal_radius(polar_pose[1], polar_pose[2])>0.2){
				cout<<"in a range 0.2m ~ 0.3m"<<endl;
				reward.data = 2.0;
			}
			else if(cal_radius(polar_pose[1], polar_pose[2])<=0.2 
					&& cal_radius(polar_pose[1], polar_pose[2])>0.1){
				cout<<"in a range 0.2m ~ 0.1m"<<endl;
				reward.data = 3.0;
			}
			else if(cal_radius(polar_pose[1], polar_pose[2])<=0.1 
					&& cal_radius(polar_pose[1], polar_pose[2])>0.05){
				cout<<"in a range 0.1m ~ 0.05m"<<endl;
				reward.data = 4.0;
			}
			else if(cal_radius(polar_pose[1], polar_pose[2])<=0.05 
					&& cal_radius(polar_pose[1], polar_pose[2])>0.02){
				cout<<"in a range 0.05m ~ 0.02m"<<endl;
				reward.data = 5.0;
			}
			else if(cal_radius(polar_pose[1], polar_pose[2])<=0.02){
				cout<<"in a range 0.02m ~ 0.00m"<<endl;
				reward.data = 10.0;
			}
			else{
				cout<<"out of range"<<endl;
				reward.data = 1.0;
			}
		}
		else{
			reward.data = 0.0;
			cout<<"you can't touch wall!!"<<endl;
		}

		cout<<"reward : "<<reward.data<<endl;
		if(pub_flag){
			pub.publish(reward);
			pub_flag = false;
		}
		cout<<endl;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
