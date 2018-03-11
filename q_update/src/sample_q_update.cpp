#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

#include <stdlib.h>
#include <time.h>

using namespace std;

int num_s = 70*70*10;
int num_a = 3*3*3;

int plob = 50;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "sample_q_update");
	ros::NodeHandle n;

	ros::Publisher pub_1 = n.advertise<std_msgs::Int64>("/state_num", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Int64>("/action_num", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/reward", 1);

	ros::Rate loop_rate(5);

	std_msgs::Int64 state_num;
	std_msgs::Int64 action_num;
	std_msgs::Float64 reward;

	while(ros::ok()){

		state_num.data = rand()%num_s;
		action_num.data = rand()%num_a;
		
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
