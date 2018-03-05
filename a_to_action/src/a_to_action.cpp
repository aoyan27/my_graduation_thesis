#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

using namespace std;

double pi = 3.141592;

int action_num;

int num_a = 3 * 3 * 3;

bool pub_flag = false;

void action_num_callback(std_msgs::Int64 msg){
	action_num = msg.data;
	// cout<<"action_num : "<<action_num<<endl;
	pub_flag = true;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "a_to_action");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/action_num", 10, action_num_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/action_type_joint1", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/action_type_joint3", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/action_type_joint5", 1);

	ros::Rate loop_rate(100);

	std_msgs::Float64 action_type_joint1;
	std_msgs::Float64 action_type_joint3;
	std_msgs::Float64 action_type_joint5;

	int i;
	
	while(ros::ok()){
		action_type_joint1.data = action_num/9;
		action_type_joint3.data = (action_num%9)/3;
		action_type_joint5.data = (action_num%3)/1;

		cout<<"action type joint1 : "<<action_type_joint1.data<<endl;
		cout<<"action type joint3 : "<<action_type_joint3.data<<endl;
		cout<<"action type joint5 : "<<action_type_joint5.data<<endl;

		cout<<endl;

		if(pub_flag){
			pub_1.publish(action_type_joint1);
			pub_2.publish(action_type_joint3);
			pub_3.publish(action_type_joint5);
			pub_flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
