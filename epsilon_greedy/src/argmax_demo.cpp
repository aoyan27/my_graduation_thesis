#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

double pi = 3.141592;
int epsilon = 0;
int num_a = 3 * 3 * 3;

std_msgs::Int64 optimum_action_num;

bool pub_flag = false;
bool epsilon_greedy_flag = true;

void epsilon_greedy_flag_callback(std_msgs::Int64 msg){
	epsilon_greedy_flag = false;
}

void optimum_action_callback(std_msgs::Int64 msg){
	optimum_action_num.data = msg.data;
	// cout<<optimum_action_num.data<<endl;
	pub_flag = true;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "epsilon_greedy");
	ros::NodeHandle n;

	ros::Subscriber sub_1 = n.subscribe("/optimum_action_num", 10, optimum_action_callback);
	ros::Subscriber sub_2 = n.subscribe("/epsilon_greedy_flag", 10, epsilon_greedy_flag_callback);

	ros::Publisher pub = n.advertise<std_msgs::Int64>("/action_num", 1);

	ros::Rate loop_rate(10);

	std_msgs::Int64 action_num;

	while(ros::ok()){
		// action_num.data = rand()%num_a;
		// cout<<rand()%100<<endl;
		if(epsilon_greedy_flag){
			if(epsilon>rand()%100){
				action_num.data = rand()%num_a;
				cout<<"random selection : "<<action_num.data<<endl;
			}
			else{
				action_num.data = optimum_action_num.data;
				cout<<"optimum action : "<<action_num.data<<endl;
			}
		}
		else{
			action_num.data = optimum_action_num.data;
			cout<<"now evoluation policy -> action :"<<action_num.data<<endl;
			epsilon_greedy_flag = true;
		}

		if(pub_flag){
			pub.publish(action_num);
			pub_flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
