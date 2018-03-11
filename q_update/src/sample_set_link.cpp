#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <gazebo_msgs/LinkState.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

int num_s = 70*70*10;
int num_a = 3*3*3;

int plob = 50;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "sample_set_link");
	ros::NodeHandle n;

	ros::Publisher pub_1 = n.advertise<std_msgs::Int64>("/state_num", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Int64>("/action_num", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/reward", 1);
	
	ros::Publisher pub_4 = n.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1);
	
	ros::Rate loop_rate(5);

	std_msgs::Int64 state_num;
	std_msgs::Int64 action_num;
	std_msgs::Float64 reward;

	// gazebo_msgs::SetLinkState setlinkstate;
	gazebo_msgs::LinkState linkstate;

	while(ros::ok()){

		state_num.data = rand()%num_s;
		action_num.data = rand()%num_a;

		// linkstate.link_name = "infant::link6";
		// linkstate.pose.position.x = 0.66;
		// linkstate.pose.position.y = 0.18;
		// linkstate.pose.position.z = 0.81;
		// linkstate.pose.orientation.x = 0.582;
		// linkstate.pose.orientation.y = 0.182;
		// linkstate.pose.orientation.z = 0.755;
		// linkstate.pose.orientation.w = -0.238;

		linkstate.link_name = "infant::link1";
		linkstate.pose.position.x = 0.42;
		linkstate.pose.position.y = 0.00;
		linkstate.pose.position.z = 0.73;
		linkstate.pose.orientation.x = 0.95;
		linkstate.pose.orientation.y = 0.30;
		linkstate.pose.orientation.z = -0.00;
		linkstate.pose.orientation.w = -0.00;

		 pub_4.publish(linkstate);
		
		cout<<state_num.data<<endl;

		if(plob>rand()%100){
			reward.data = 10.0;
		}
		else{
			reward.data = 0.0;
		}

		pub_1.publish(state_num);
		pub_2.publish(action_num);
		pub_3.publish(reward);

		
		loop_rate.sleep();
	}
	return 0;
}
