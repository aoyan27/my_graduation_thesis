#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <gazebo_msgs/GetLinkState.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>

#include <reward_calculation/reward.h>

using namespace std;

double pi = 3.141592;

bool pub_flag = false;

double r;

double polar_pose[3];
double arm_pose[3];

// double distance_now;
// double distance_past;

double arm_pose_init[3]={0.668, 0.000, 0.873};

double target_init[3] = {0.680, 0.000, 0.900};

sensor_msgs::PointCloud target;

std_msgs::Float64 distance_arm;
std_msgs::Float64 radius_arm;

ros::Publisher pub_1;
ros::Publisher pub_2;

void target_point_callback(sensor_msgs::PointCloud msg){
	target.header.stamp = ros::Time::now();
	target = msg;
	// cout<<"subscribe : "<<target<<endl;
}


double celi2(double dIn, int nLen){
	double dOut;

	dOut = dIn * pow(10.0, nLen);
	dOut = (double)(int)(dOut + 0.9);

	return dOut * pow(10.0, -nLen);
}

double	distance_past = celi2(sqrt(pow(target_init[0]-arm_pose_init[0], 2) 
			+ pow(target_init[1]-arm_pose_init[1], 2) + pow(target_init[2]-arm_pose_init[2], 2)), 5);

double	distance_now =celi2(sqrt(pow(target_init[0]-arm_pose_init[0], 2) 
			+ pow(target_init[1]-arm_pose_init[1], 2) + pow(target_init[2]-arm_pose_init[2], 2)), 5);


// double arm_pose_init[3];

// double target_pose[3];

// double celi2(double dIn, int nLen){
	// double dOut;

	// dOut = dIn * pow(10.0, nLen);
	// dOut = (double)(int)(dOut + 0.9);

	// return dOut * pow(10.0, -nLen);
// }

double cal_radius(double y, double z){
	double dist_r;
	dist_r = sqrt(pow(y, 2) + pow(z, 2));

	return dist_r;
}

bool reward_server_callback(reward_calculation::reward::Request &req,
							reward_calculation::reward::Response &res){
	pub_flag = true;

	polar_pose[0] = arm_pose[0] - target.points[0].x;
	polar_pose[1] = arm_pose[1] - target.points[0].y;
	polar_pose[2] = arm_pose[2] - target.points[0].z;

	r = cal_radius(polar_pose[1], polar_pose[2]);

	cout<<"arm_pose x : "<<arm_pose[0]<<endl;
	cout<<"arm_pose y : "<<arm_pose[1]<<endl;
	cout<<"arm_pose z : "<<arm_pose[2]<<endl<<endl;
	cout<<"polar arm_pose x : "<<polar_pose[0]<<endl;
	cout<<"polar arm_pose y : "<<polar_pose[1]<<endl;
	cout<<"polar arm_pose z : "<<polar_pose[2]<<endl;
	cout<<"target x : "<<target.points[0].x<<endl;
	cout<<"target y : "<<target.points[0].y<<endl;
	cout<<"target z : "<<target.points[0].z<<endl;
	cout<<"r : "<<r<<endl;

	distance_now = celi2(sqrt(pow(target.points[0].x-arm_pose[0], 2)
			+ pow(target.points[0].y-arm_pose[1], 2) + pow(target.points[0].z-arm_pose[2], 2)), 5);
		
	if(req.request_reward == 0){
		distance_past = distance_now;
		cout<<"distance initialize!!!!"<<endl;
	}
	cout<<"distance_past : "<<distance_past<<endl;
	cout<<"distance_now : "<<distance_now<<endl;

	cout<<"diff : "<<distance_now -distance_past<<endl;

	if((distance_now - distance_past) < 0.0){
		res.reward = 0.0;
		cout<<"arm approached!! and ";
		distance_past = distance_now;
	}
	else if((distance_now - distance_past) > 0.0){
		res.reward = -0.5;
		cout<<"arm leaved!! and ";
		distance_past = distance_now;
	}
	else{
		res.reward = 0.0;
		cout<<"arm doen't move!! and ";
		distance_past = distance_now;
	}

	cout<<"distance_now : "<<distance_now<<endl;
	cout<<"distance_past : "<<distance_past<<endl;
		
	// cout<<"polar arm_pose x : "<<polar_pose[0]<<endl;
	// cout<<"polar arm_pose y : "<<polar_pose[1]<<endl;
	// cout<<"polar arm_pose z : "<<polar_pose[2]<<endl;
	if(arm_pose[0] >= target.points[0].x){
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
		if(r <= 0.04){
			cout<<"in a range 0.02m ~ 0.00m"<<endl;
			res.reward = 1.0;
		}
	}
	else{
		res.reward += 0.0;
		cout<<"you can't touch wall!!"<<endl;
	}
	
	cout<<"reward : "<<res.reward<<endl;

	cout<<endl;
	
	distance_arm.data = distance_now;
	radius_arm.data = r;

	pub_1.publish(distance_arm);
	pub_2.publish(radius_arm);
	
	ROS_INFO("request: request_reward=%ld", req.request_reward);
	ROS_INFO("sending back response: [%lf]", res.reward);
	return true;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "reward_calculation_server_tis");
	ros::NodeHandle n;

	ros::ServiceClient client;

	string linkName = "infant::link8";
	gazebo_msgs::GetLinkState getlinkstate;
	ros::ServiceServer service = n.advertiseService("reward", reward_server_callback);
	
	client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
	
	ros::Subscriber sub = n.subscribe("/target_point", 10, target_point_callback);

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("/vis_target_point2", 1);

	pub_1 = n.advertise<std_msgs::Float64>("/distance", 1);
	pub_2 = n.advertise<std_msgs::Float64>("/radius", 1);

	getlinkstate.request.link_name = linkName;
	ros::Rate loop_rate(100);

	float polar_pose_vis[3];

	target.header.frame_id ="/base_link";
	target.header.stamp = ros::Time::now();
	target.points.resize(1);
	target.points[0].x = 0.680;
	target.points[0].y = 0.000;
	target.points[0].z = 0.900;

	double distance_vis = 0.0;
	
	while(ros::ok()){
		client.call(getlinkstate);
		arm_pose[0] = getlinkstate.response.link_state.pose.position.x;
		arm_pose[1] = getlinkstate.response.link_state.pose.position.y;
		arm_pose[2] = getlinkstate.response.link_state.pose.position.z;
	
		polar_pose_vis[0] = arm_pose[0] - target.points[0].x;
		polar_pose_vis[1] = arm_pose[1] - target.points[0].y;
		polar_pose_vis[2] = arm_pose[2] - target.points[0].z;

		// cout<<"r : "<<cal_radius(polar_pose_vis[1], polar_pose_vis[2])<<endl;
	
		cout<<"x : "<<getlinkstate.response.link_state.pose.position.x<<endl;
		cout<<"y : "<<getlinkstate.response.link_state.pose.position.y<<endl;
		cout<<"z : "<<getlinkstate.response.link_state.pose.position.z<<endl;
	
		// cout<<endl;	
		// ROS_INFO("reward_calculation_sever start.");

		cout<<"target x : "<<target.points[0].x<<endl;
		cout<<"target y : "<<target.points[0].y<<endl;
		cout<<"target z : "<<target.points[0].z<<endl;

		// cout<<"distance_past : "<<distance_past<<endl;
		// cout<<"distance_now : "<<distance_now<<endl;

		// cout<<"diff : "<<distance_now -distance_past<<endl;
		distance_vis = celi2(sqrt(pow(target.points[0].x-arm_pose[0], 2) 
					+ pow(target.points[0].y-arm_pose[1], 2) + pow(target.points[0].z-arm_pose[2], 2)), 5);

		cout<<"distance_vis : "<<distance_vis<<endl;
		if(pub_flag){
			// distance.data = distance_now;
			// radius.data = r;
			// pub_1.publish(distance);
			// pub_2.publish(radius);
			pub.publish(target);
			pub_flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
