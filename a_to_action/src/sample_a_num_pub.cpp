#include <ros/ros.h>
#include <std_msgs/Int64.h>

using namespace std;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "sample_a_num_pub");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int64>("/action_num", 1);

	ros::Rate loop_rate(5);

	std_msgs::Int64 action_num;

	while(ros::ok()){
		action_num.data = 15;

		pub.publish(action_num);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
