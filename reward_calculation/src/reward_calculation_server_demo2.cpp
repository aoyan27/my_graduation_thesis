#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <gazebo_msgs/GetLinkState.h>
#include <math.h>

#include <reward_calculation/reward.h>

using namespace std;

double pi = 3.141592;

bool pub_flag = false;


double polar_pose[3];
double arm_pose[3];

double distance_now;
double distance_past;


double arm_pose_init[3];

double target_pose[3];

double celi2(double dIn, int nLen){
	double dOut;

	dOut = dIn * pow(10.0, nLen);
	dOut = (double)(int)(dOut + 0.9);

	return dOut * pow(10.0, -nLen);
}

double cal_radius(double y, double z){
	double dist_r;
	dist_r = sqrt(pow(y, 2) + pow(z, 2));

	return dist_r;
}

bool reward_server_callback(reward_calculation::reward::Request &req,
							reward_calculation::reward::Response &res){


	target_pose[0] = 0.88;
	target_pose[1] = 0.00;
	target_pose[2] = 0.84;

	polar_pose[0] = arm_pose[0] - target_pose[0];
	polar_pose[1] = arm_pose[1] - target_pose[1];
	polar_pose[2] = arm_pose[2] - target_pose[2];

	// cout<<"r : "<<cal_radius(polar_pose[1], polar_pose[2])<<endl;;
	distance_now = celi2(sqrt(pow(target_pose[0]-arm_pose[0], 2)
			+ pow(target_pose[1]-arm_pose[1], 2) + pow(target_pose[2]-arm_pose[2], 2)), 2);

	cout<<"distance_past : "<<distance_past<<endl;
	cout<<"distance_now : "<<distance_now<<endl;

	cout<<"diff : "<<distance_now -distance_past<<endl;

	/*if((distance_now - distance_past) > 0.0){
		res.reward = 1.0;
		cout<<"arm approached!! and ";
	}
	else{
		res.reward = 0.0;
		cout<<"arm leaved!! and ";
	}*/
	// cout<<"distance_now : "<<distance_now<<endl;
	// cout<<"distance_past : "<<distance_past<<endl;
		
	
	if(arm_pose[0] >= 0.91){
		cout<<"you can touch wall!!"<<endl;
		/*if(cal_radius(polar_pose[1], polar_pose[2])<=0.3 
				&& cal_radius(polar_pose[1], polar_pose[2])>0.2){
			cout<<"in a range 0.2m ~ 0.3m"<<endl;
			reward.data += 2.0;
		}
		else if(cal_radius(polar_pose[1], polar_pose[2])<=0.2 
				&& cal_radius(polar_pose[1], polar_pose[2])>0.1){
			cout<<"in a range 0.2m ~ 0.1m"<<endl;
			reward.data += 3.0;
		}
		else if(cal_radius(polar_pose[1], polar_pose[2])<=0.1 
				&& cal_radius(polar_pose[1], polar_pose[2])>0.05){
			cout<<"in a range 0.1m ~ 0.05m"<<endl;
			reward.data += 4.0;
		}
		else if(cal_radius(polar_pose[1], polar_pose[2])<=0.05 
				&& cal_radius(polar_pose[1], polar_pose[2])>0.02){
			cout<<"in a range 0.05m ~ 0.02m"<<endl;
			reward.data += 5.0;
		}
		else if(cal_radius(polar_pose[1], polar_pose[2])<=0.02){
			cout<<"in a range 0.02m ~ 0.00m"<<endl;
			reward.data += 10.0;
		}
		else{
			cout<<"out of range"<<endl;
			reward.data += 1.0;
		}*/
		if(cal_radius(polar_pose[1], polar_pose[2])<=0.05){
			cout<<"in a range 0.05m ~ 0.00m"<<endl;
			res.reward = 100.0;
		}
	}
	else{
		res.reward += 0.0;
		cout<<"you can't touch wall!!"<<endl;
	}
	
	cout<<"reward : "<<res.reward<<endl;

	cout<<endl;

	distance_past = distance_now;

	ROS_INFO("request: request_reward=%ld", req.request_reward);
	ROS_INFO("sending back response: [%lf]", res.reward);
	return true;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "reward_cal_server");
	ros::NodeHandle n;

	ros::ServiceClient client;

	string linkName = "infant::link8";
	gazebo_msgs::GetLinkState getlinkstate;
	ros::ServiceServer service = n.advertiseService("reward", reward_server_callback);
	
	client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
	
	getlinkstate.request.link_name = linkName;
	ros::Rate loop_rate(100);

	arm_pose_init[0] = 0.84;
	arm_pose_init[1] = 0.23;
	arm_pose_init[2] = 0.84;

	target_pose[0] = 0.88;
	target_pose[1] = 0.00;
	target_pose[2] = 0.84;

	distance_past = celi2(sqrt(pow(target_pose[0]-arm_pose_init[0], 2) 
			+ pow(target_pose[1]-arm_pose_init[1], 2) + pow(target_pose[2]-arm_pose_init[2], 2)), 2);
	
	while(ros::ok()){
		client.call(getlinkstate);
		arm_pose[0] = getlinkstate.response.link_state.pose.position.x;
		arm_pose[1] = getlinkstate.response.link_state.pose.position.y;
		arm_pose[2] = getlinkstate.response.link_state.pose.position.z;
	
		cout<<"x : "<<getlinkstate.response.link_state.pose.position.x<<endl;
		cout<<"y : "<<getlinkstate.response.link_state.pose.position.y<<endl;
		cout<<"z : "<<getlinkstate.response.link_state.pose.position.z<<endl;
		cout<<endl;	
		// ROS_INFO("reward_calculation_sever start.");
	
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
