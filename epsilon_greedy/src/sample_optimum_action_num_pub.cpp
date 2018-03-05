#include <ros/ros.h>
#include <std_msgs/Int64.h>

using namespace std;

double pi = 3.141592;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "sample_optimum_a_num_pub");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int64>("/optimum_action_num", 1);

	ros::Rate loop_rate(5);

	std_msgs::Int64 optimum_action_num;

	int count = 0;

	while(ros::ok()){
		optimum_action_num.data = 16;

		cout<<"count : "<<count<<endl;

		if(count == 25){
			cout<<"publish"<<endl;
			pub.publish(optimum_action_num);
			count = 0;
		}

		count++;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
