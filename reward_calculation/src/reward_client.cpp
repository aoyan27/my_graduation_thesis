#include <ros/ros.h>
#include <reward_calculation/reward.h>
#include <cstdlib>

int main(int argc, char **argv){
	ros::init(argc, argv, "reward_client");
	// if(argc != 3){
		// ROS_INFO("usage: reward_client X Y");
		// return -1;
	// }

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<reward_calculation::reward>("reward");
	reward_calculation::reward srv;
	srv.request.request_reward = 0;
	ros::Rate loop_rate(10);
	while(ros::ok()){
		if(client.call(srv)){
			ROS_INFO("reward: %lf", srv.response.reward);
		}
		else{
			ROS_ERROR("Failed to call sevice reward_calculation_server");
			return 1;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
