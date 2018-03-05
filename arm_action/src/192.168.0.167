#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <math.h>
#include <stdio.h>

using namespace std;

double pi = 3.141592;

int init_joint1 = 0;
int init_joint2 = -50;
int init_joint3 = 105;

int init_joint[3];

float upper_limit_joint[3];
float lower_limit_joint[3];

std_msgs::Float64 action_type_joint1;
std_msgs::Float64 action_type_joint2;
std_msgs::Float64 action_type_joint3;

std_msgs::Float64 joint1_pose;
std_msgs::Float64 joint2_pose;
std_msgs::Float64 joint3_pose;

int num_joint = 3;

int num_episode = 0;
int num_step = 0;

bool pub_action_type_flag1 = false;
bool pub_action_type_flag2 = false;
bool pub_action_type_flag3 = false;

bool pub_pose_com_flag1 = false;
bool pub_pose_com_flag2 = false;
bool pub_pose_com_flag3 = false;

void num_episode_callback(std_msgs::Int64 msg){
	num_episode = msg.data;
}

void num_step_callback(std_msgs::Int64 msg){
	num_step = msg.data;
}

void action_type_joint1_callback(std_msgs::Float64 msg){
	action_type_joint1.data = msg.data;
	cout<<"type joint1 : "<<action_type_joint1.data<<endl;
	pub_action_type_flag1 = true;
}

void action_type_joint2_callback(std_msgs::Float64 msg){
	action_type_joint2.data = msg.data;
	cout<<"type joint2 : "<<action_type_joint2.data<<endl;
	pub_action_type_flag2 = true;
}

void action_type_joint3_callback(std_msgs::Float64 msg){
	action_type_joint3.data = msg.data;
	cout<<"type joint3 : "<<action_type_joint3.data<<endl;
	pub_action_type_flag3 = true;
}

void joint1_pose_callback(std_msgs::Float64 msg){
	joint1_pose.data = msg.data;
	cout<<"pose joint1 : "<<joint1_pose.data<<endl;
	pub_pose_com_flag1 = true;
}

void joint2_pose_callback(std_msgs::Float64 msg){
	joint2_pose.data = msg.data;
	cout<<"pose joint2 : "<<joint2_pose.data<<endl;
	pub_pose_com_flag2 = true;
}

void joint3_pose_callback(std_msgs::Float64 msg){
	joint3_pose.data = msg.data;
	cout<<"pose joint3 : "<<joint3_pose.data<<endl;
	pub_pose_com_flag3 = true;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "arm_action_tis");
	ros::NodeHandle n;

	ros::Subscriber sub_episode = n.subscribe("/num_episode", 10, num_episode_callback);
	ros::Subscriber sub_step = n.subscribe("/num_step", 10, num_step_callback);

	ros::Subscriber sub_1 = n.subscribe("/action_type_joint1",10, action_type_joint1_callback);
	ros::Subscriber sub_2 = n.subscribe("/action_type_joint2",10, action_type_joint2_callback);
	ros::Subscriber sub_3 = n.subscribe("/action_type_joint3",10, action_type_joint3_callback);
	ros::Subscriber sub_4 = n.subscribe("/joint1_pose",10, joint1_pose_callback);
	ros::Subscriber sub_5 = n.subscribe("/joint2_pose",10, joint2_pose_callback);
	ros::Subscriber sub_6 = n.subscribe("/joint3_pose",10, joint3_pose_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/joint1_pose_com", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/joint2_pose_com", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/joint3_pose_com", 1);

	ros::Rate loop_rate(100);

	std_msgs::Float64 joint1_pose_com;
	std_msgs::Float64 joint2_pose_com;
	std_msgs::Float64 joint3_pose_com;

	float action_type[3];
	double joint_pose[3];
	
	int i;

	init_joint[0] = init_joint1; 
	init_joint[1] = init_joint2;
	init_joint[2] = init_joint3;

	for(int i =0;i<3;i++){
		upper_limit_joint[i] = (float)init_joint[i];
		lower_limit_joint[i] = (float)init_joint[i];
	}
	upper_limit_joint[0] += 10.0;
	upper_limit_joint[1] += 0.0;
	upper_limit_joint[2] += 0.0;
	cout<<"upper_limit(joint1) : "<<upper_limit_joint[0]<<endl;
	cout<<"upper_limit(joint2) : "<<upper_limit_joint[1]<<endl;
	cout<<"upper_limit(joint3) : "<<upper_limit_joint[2]<<endl;

	lower_limit_joint[0] -= 10.0;
	lower_limit_joint[1] -= 10.0;
	lower_limit_joint[2] -= 10.0;
	cout<<"lower_limit(joint1) : "<<lower_limit_joint[0]<<endl;
	cout<<"lower_limit(joint2) : "<<lower_limit_joint[1]<<endl;
	cout<<"lower_limit(joint3) : "<<lower_limit_joint[2]<<endl;

	
	while(ros::ok()){
		if((pub_action_type_flag1 && pub_action_type_flag2 && pub_action_type_flag3) 
				&& (pub_pose_com_flag1 && pub_pose_com_flag2 && pub_pose_com_flag3)){
			cout<<"num_step : "<<num_step<<endl;
			
			action_type[0] = action_type_joint1.data;
			action_type[1] = action_type_joint2.data;
			action_type[2] = action_type_joint3.data;

			joint_pose[0] = joint1_pose.data;
			joint_pose[1] = joint2_pose.data;
			joint_pose[2] = joint3_pose.data;

			// cout<<"joint1_pose : "<<joint_pose[0]<<endl;
			// cout<<"joint2_pose : "<<joint_pose[1]<<endl;
			// cout<<"joint3_pose : "<<joint_pose[2]<<endl;

			for(i=0;i<num_joint;i++){
				if(action_type[i]==0.0){
					cout<<"joint"<<i+1<<" action type : 0.0 -> joint_angle stay"<<endl;
					joint_pose[i] += 0.0;
				}
				else if(action_type[i]==1.0){//+1deg
					if(i == 0){
						// cout<<upper_limit_joint[i] - (float)init_joint[i]<<endl;
						if(1.0*(float)init_joint[i]<=175.0){
							if(joint_pose[i]>=1.0*upper_limit_joint[i]){
								cout<<"joint"<<i+1<<" action type : 1.0 but joint_angle upper limit!"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								if(joint_pose[i]<175.0){
									cout<<"joint"<<i+1<<" action type : 1.0 -> joint_angle increase 1 deg!"<<endl;
									joint_pose[i] += 1.0;
								}
								else{
									cout<<"joint"<<i+1<<" action type : 1.0 but joint_angle upper limit!"<<endl;
									joint_pose[i] += 0.0;
									
								}
							}
						}
						else{
							ROS_ERROR("Initialize joint_%d failed. Please fall within the scope!!", i+1);
						}
					}
					else if(i == 1){
						if(1.0*(float)init_joint[i]<=95.0){
							if(joint_pose[i]>=1.0*upper_limit_joint[i]){
								cout<<"joint"<<i+1<<" action type : 1.0 but joint_angle upper limit!"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								if(joint_pose[i]<95.0){
									cout<<"joint"<<i+1<<" action type : 1.0 -> joint_angle increase 1 deg!"<<endl;
									joint_pose[i] += 1.0;
								}
								else{
									cout<<"joint"<<i+1<<" action type : 1.0 but joint_angle upper limit!"<<endl;
									joint_pose[i] += 0.0;
									
								}
							}
						}
						else{
							ROS_ERROR("Initialize joint_%d failed. Please fall within the scope!!", i+1);
						}	
					}
					else{//i == 2
						if(1.0*(float)init_joint[i]<=150.0){
							if(joint_pose[i]>=1.0*upper_limit_joint[i]){
								cout<<"joint"<<i+1<<" action type : 1.0 but joint_angle upper limit!"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								if(joint_pose[i]<150.0){
									cout<<"joint"<<i+1<<" action type : 1.0 -> joint_angle increase 1 deg!"<<endl;
									joint_pose[i] += 1.0;
								}
								else{
									cout<<"joint"<<i+1<<" action type : 1.0 but joint_angle upper limit!!"<<endl;
									joint_pose[i] += 0.0;
									
								}
							}
						}
						else{
							ROS_ERROR("Initialize joint_%d failed. Please fall within the scope!!", i+1);
						}
					}
				}
				else{//action_type[i]==2.0 //-1deg
					if(i == 0){
						// cout<<upper_limit_joint[i] - (float)init_joint[i]<<endl;
						if(1.0*(float)init_joint[i]>=-175.0){
							if(joint_pose[i]<=1.0*lower_limit_joint[i]){
								cout<<"joint"<<i+1<<" action type : 2.0 but joint_angle lower limit!"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								if(joint_pose[i]>-175.0){
									cout<<"joint"<<i+1<<" action type : 2.0 -> joint_angle decrease 1 deg!"<<endl;
									joint_pose[i] -= 1.0;
								}
								else{
									cout<<"joint"<<i+1<<" action type : 2.0 but joint_angle lower limit!"<<endl;
									joint_pose[i] += 0.0;
								}
							}
						}
						else{
							ROS_ERROR("Initialize joint_%d failed. Please fall within the scope!!", i+1);
						}
					}
					else if(i == 1){
						if(1.0*(float)init_joint[i]>=-95.0){
							if(joint_pose[i]<=1.0*lower_limit_joint[i]){
								cout<<"joint"<<i+1<<" action type : 2.0 but joint_angle lower limit!"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								if(joint_pose[i]>-95.0){
									cout<<"joint"<<i+1<<" action type : 2.0 -> joint_angle decrease 1 deg!"<<endl;
									joint_pose[i] -= 1.0;
								}
								else{
									cout<<"joint"<<i+1<<" action type : 2.0 but joint_angle lower limit!"<<endl;
									joint_pose[i] += 0.0;
								}
							}
						}
						else{
							ROS_ERROR("Initialize joint_%d failed. Please fall within the scope!!", i+1);
						}	
					}
					else{//i == 2
						if(1.0*(float)init_joint[i]>=-10.0){
							if(joint_pose[i]<=1.0*lower_limit_joint[i]){
								cout<<"joint"<<i+1<<" action type : 2.0 but joint_angle lower limit!"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								if(joint_pose[i]>-10.0){
									cout<<"joint"<<i+1<<" action type : 2.0 -> joint_angle decrease 1 deg!"<<endl;
									joint_pose[i] -= 1.0;
								}
								else{
									cout<<"joint"<<i+1<<" action type : 2.0 but joint_angle lower limit!!"<<endl;
									joint_pose[i] += 0.0;
								}
							}
						}
						else{
							ROS_ERROR("Initialize joint_%d failed. Please fall within the scope!!", i+1);
						}
					}
					
				}

			}

			cout<<endl;

			joint1_pose_com.data = joint_pose[0];
			joint2_pose_com.data = joint_pose[1];
			joint3_pose_com.data = joint_pose[2];
			cout<<"joint1_pose_com : "<<joint1_pose_com.data<<endl;
			cout<<"joint2_pose_com : "<<joint2_pose_com.data<<endl;
			cout<<"joint3_pose_com : "<<joint3_pose_com.data<<endl;

			cout<<endl;
			
			pub_1.publish(joint1_pose_com);
			pub_2.publish(joint2_pose_com);
			pub_3.publish(joint3_pose_com);
			pub_action_type_flag1 = false;
			pub_action_type_flag2 = false;
			pub_action_type_flag3 = false;
			pub_pose_com_flag1 = false;
			pub_pose_com_flag2 = false;
			pub_pose_com_flag3 = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
