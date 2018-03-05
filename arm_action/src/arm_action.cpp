#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

using namespace std;

double pi = 3.141592;

int init_state_joint1 = -35;
int init_state_joint3 = 40;
int init_state_joint5 = 30;
  
int init_state = 17600;
int init_joint[3];

std_msgs::Float64 action_type_joint1;
std_msgs::Float64 action_type_joint3;
std_msgs::Float64 action_type_joint5;

std_msgs::Float64 joint1_pose;
std_msgs::Float64 joint3_pose;
std_msgs::Float64 joint5_pose;

int num_joint = 3;

bool pub_flag1 = false;
bool pub_flag2 = false;

void action_type_joint1_callback(std_msgs::Float64 msg){
	action_type_joint1.data = msg.data;
	// cout<<"type joint1 : "<<action_type_joint1.data<<endl;
	pub_flag1 = true;
}

void action_type_joint3_callback(std_msgs::Float64 msg){
	action_type_joint3.data = msg.data;
	// cout<<"type joint3 : "<<action_type_joint3.data<<endl;
}

void action_type_joint5_callback(std_msgs::Float64 msg){
	action_type_joint5.data = msg.data;
	// cout<<"type joint5 : "<<action_type_joint5.data<<endl;
}

void joint1_pose_callback(std_msgs::Float64 msg){
	joint1_pose.data = msg.data;
	// cout<<"pose joint1 : "<<joint1_pose.data<<endl;
	pub_flag2 = true;
}

void joint3_pose_callback(std_msgs::Float64 msg){
	joint3_pose.data = msg.data;
	// cout<<"pose joint3 : "<<joint3_pose.data<<endl;
}

void joint5_pose_callback(std_msgs::Float64 msg){
	joint5_pose.data = msg.data;
	// cout<<"pose joint5 : "<<joint5_pose.data<<endl;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "arm_action");
	ros::NodeHandle n;


	ros::Subscriber sub_1 = n.subscribe("/action_type_joint1",10, action_type_joint1_callback);
	ros::Subscriber sub_2 = n.subscribe("/action_type_joint3",10, action_type_joint3_callback);
	ros::Subscriber sub_3 = n.subscribe("/action_type_joint5",10, action_type_joint5_callback);
	ros::Subscriber sub_4 = n.subscribe("/joint1_pose",10, joint1_pose_callback);
	ros::Subscriber sub_5 = n.subscribe("/joint3_pose",10, joint3_pose_callback);
	ros::Subscriber sub_6 = n.subscribe("/joint5_pose",10, joint5_pose_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/joint1_pose_com", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/joint3_pose_com", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/joint5_pose_com", 1);

	ros::Rate loop_rate(100);

	std_msgs::Float64 joint1_pose_com;
	std_msgs::Float64 joint3_pose_com;
	std_msgs::Float64 joint5_pose_com;

	float action_type[3];
	float joint_pose[3];
	
	int i;

	// init_joint[0] = init_state/700 + init_state_joint1; 
	// init_joint[1] = (init_state%700)/10 + init_state_joint3; 
	// init_joint[2] = (init_state%10)/1 + init_state_joint5; 

	// cout<<"init_joint1 : "<<init_joint[0]<<endl;
	// cout<<"init_joint3 : "<<init_joint[1]<<endl;
	// cout<<"init_joint5 : "<<init_joint[2]<<endl;

	while(ros::ok()){
		init_joint[0] = init_state/700 + init_state_joint1; 
		init_joint[1] = (init_state%700)/10 + init_state_joint3; 
		init_joint[2] = (init_state%10)/1 + init_state_joint5; 
		// cout<<"init_joint1 : "<<init_joint[0]<<endl;
		// cout<<"init_joint3 : "<<init_joint[1]<<endl;
		// cout<<"init_joint5 : "<<init_joint[2]<<endl;
	
		if(pub_flag1 && pub_flag2){
			action_type[0] = action_type_joint1.data;
			action_type[1] = action_type_joint3.data;
			action_type[2] = action_type_joint5.data;

			joint_pose[0] = joint1_pose.data;
			joint_pose[1] = joint3_pose.data;
			joint_pose[2] = joint5_pose.data;

			cout<<"joint1_pose : "<<joint_pose[0]<<endl;
			cout<<"joint3_pose : "<<joint_pose[1]<<endl;
			cout<<"joint5_pose : "<<joint_pose[2]<<endl;

			for(i=0;i<num_joint;i++){
				if(action_type[i]==0.0){
					cout<<"action type : 0.0 -> joint_angle stay"<<endl;
					joint_pose[i] += 0.0;

				}
				else if(action_type[i]==1.0){
					if(i==0){
						if((1.0*(float)fabs(init_joint[0]))<=34.0){
							if(joint_pose[0]>=1.0*(float)fabs(init_joint[0])){
								cout<<"action type : 1.0 but joint_angle upper limit!"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								cout<<"action type : 1.0 -> joint_angle increase 1 deg!"<<endl;
								joint_pose[i] += 1.0;
								// cout<<joint_pose[i]<<endl;
							}
						}
						else{
							if(joint_pose[0]>=34.0){
								cout<<"action type : 2.0 but joint_angle laower limit"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								cout<<"action type : 2.0 -> joint_angle decrease 1 deg"<<endl;
								joint_pose[i] -= 1.0;
								// cout<<joint_pose[i]<<endl;
							}	
						}
					}
					if(i==1){
						if(init_joint[1] > init_state_joint3){
							if(joint_pose[i]>=(float)init_joint[1]){
								cout<<"action type : 1.0 but joint_angle upper limit!"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								cout<<"action type : 1.0 -> joint_angle increase 1 deg!"<<endl;
								joint_pose[i] += 1.0;
								// cout<<joint_pose[i]<<endl;
							}
						}
						else{	
							if(joint_pose[i]>=109.0){
								cout<<"action type : 1.0 but joint_angle upper limit"<<endl;
								joint_pose[i] += 0.0;
							}
							else{
								cout<<"action type : 1.0 -> joint_angle increase 1 deg"<<endl;
								joint_pose[i] += 1.0;
								// cout<<joint_pose[i]<<endl;
							}
						}
					}
					if(i==2){
						if(joint_pose[i]>=39.0){
							cout<<"action type : 1.0 but joint_angle upper limit"<<endl;
							joint_pose[i] += 0.0;
						}
						else{
							cout<<"action type : 1.0 -> joint_angle increase 1 deg"<<endl;
							joint_pose[i] += 1.0;
							// cout<<joint_pose[i]<<endl;
						}
					}
				}
				else{
					if(i==0){
						if(joint_pose[0]<=-1.0*(float)fabs(init_joint[0])){
							cout<<"action type : 2.0 but joint_angle laower limit!"<<endl;
							joint_pose[i] += 0.0;
						}
						else{
							cout<<"action type : 2.0 -> joint_angle decrease 1 deg"<<endl;
							joint_pose[i] -= 1.0;
							// cout<<joint_pose[i]<<endl;
						}
					}
					if(i==1){
						if(joint_pose[i]<=40.0){
							cout<<"action type : 2.0 but joint_angle lower limit"<<endl;
							joint_pose[i] += 0.0;
						}
						else{
							cout<<"action type : 2.0 -> joint_angle decrease 1 deg"<<endl;
							joint_pose[i] -= 1.0;
							// cout<<joint_pose[i]<<endl;
						}
					}
					if(i==2){
						if(joint_pose[i]<=30.0){
							cout<<"action type : 2.0 but joint_angle lower limit"<<endl;
							joint_pose[i] += 0.0;
						}
						else{
							cout<<"action type : 2.0 -> joint_angle decrease 1 deg"<<endl;
							joint_pose[i] -= 1.0;
							// cout<<joint_pose[i]<<endl;
						}
					}
				}
			}

			cout<<endl;

			joint1_pose_com.data = joint_pose[0];
			joint3_pose_com.data = joint_pose[1];
			joint5_pose_com.data = joint_pose[2];
			cout<<"joint1_pose_com : "<<joint1_pose_com.data<<endl;
			cout<<"joint3_pose_com : "<<joint3_pose_com.data<<endl;
			cout<<"joint5_pose_com : "<<joint5_pose_com.data<<endl;

			cout<<endl;
			
			/*if(pub_flag1 && pub_flag2){
				pub_1.publish(joint1_pose_com);
				pub_2.publish(joint3_pose_com);
				pub_3.publish(joint5_pose_com);
				pub_flag1 = false;
				pub_flag2 = false;
			}*/
			pub_1.publish(joint1_pose_com);
			pub_2.publish(joint3_pose_com);
			pub_3.publish(joint5_pose_com);
			pub_flag1 = false;
			pub_flag2 = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
