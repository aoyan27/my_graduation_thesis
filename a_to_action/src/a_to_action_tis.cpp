#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <math.h>

using namespace std;

double pi = 3.141592;

int action_num;

int num_episode;
int num_step;

int num_a = 3 * 3 * 3;

bool pub_flag = false;

void num_episode_callback(std_msgs::Int64 msg){
	num_episode = msg.data;
	// cout<<"num_episode : "<<num_episode<<endl;
}

void num_step_callback(std_msgs::Int64 msg){
	num_step = msg.data;
	// cout<<"num_step : "<<num_step<<endl;
}

void action_num_callback(std_msgs::Int64 msg){
	action_num = msg.data;
	// cout<<"action_num : "<<action_num<<endl;
	pub_flag = true;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "a_to_action_tis");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/action_num", 10, action_num_callback);
	
	ros::Subscriber sub_episode = n.subscribe("/num_episode", 10, num_episode_callback);
	ros::Subscriber sub_step = n.subscribe("/num_step", 10, num_step_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/action_type_joint1", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/action_type_joint2", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/action_type_joint3", 1);

	ros::Rate loop_rate(100);

	std_msgs::Float64 action_type_joint1;
	std_msgs::Float64 action_type_joint2;
	std_msgs::Float64 action_type_joint3;

	int i;
	
	while(ros::ok()){
		if(pub_flag){
			action_type_joint1.data = action_num/(int)pow(3.0, 2.0);
			action_type_joint2.data = (action_num%(int)pow(3.0, 2.0))/(int)pow(3.0, 1.0);
			action_type_joint3.data = (action_num%(int)pow(3.0, 1.0))/(int)pow(3.0, 0.0);
	
			cout<<"num_episode : "<<num_episode<<endl;
			cout<<"num_step : "<<num_step<<endl;
			cout<<"action_num : "<<action_num<<endl;
			
			cout<<"action type joint1 : "<<action_type_joint1.data<<endl;
			cout<<"action type joint2 : "<<action_type_joint2.data<<endl;
			cout<<"action type joint3 : "<<action_type_joint3.data<<endl;
	
			cout<<endl;

			pub_1.publish(action_type_joint1);
			pub_2.publish(action_type_joint2);
			pub_3.publish(action_type_joint3);
			pub_flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
