#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <stdio.h>

#include <joint_to_s/joint_state.h>

using namespace std;

double pi = 3.141592;

int init_state_joint1 = -35;
int init_state_joint3 = 40;
int init_state_joint5 = 30;

const int num_s = 70*70*10;
const int num_a = 3*3*3;

float alpha = 0.5;
float gamma_1 = 0.9;

int a;

bool wait_flag = false;
bool select_action_pub_flag = false;
bool q_update_flag = false;
bool state_observation_flag = true;
// bool state_observation_flag_demo = true;


float joint_state[3];

std_msgs::Int64 optimum_action;

float reward;

void state_observation_flag_callback(std_msgs::Float64 msg){
	state_observation_flag = true;
}

void joint_states_callback(sensor_msgs::JointState msg){
	joint_state[0] = msg.position[0] / pi * 180;
	joint_state[1] = msg.position[2] / pi * 180;
	joint_state[2] = msg.position[4] / pi * 180;

	// cout<<"joint1 : "<<joint_state[0]<<endl;
	// cout<<"joint3 : "<<joint_state[1]<<endl;
	// cout<<"joint5 : "<<joint_state[2]<<endl;	
}

void action_num_callback(std_msgs::Int64 msg){
	a = msg.data;
	// q_update_flag = true;
}

void reward_callback(std_msgs::Float64 msg){
	reward = msg.data;
}

float max_Qval(int s, int a_num, float Qtable[num_s][num_a]){
	float max;
	int i=0;

	max = Qtable[s][0];
	for(i=1;i<a_num;i++){
		if(Qtable[s][i]>max){
			max = Qtable[s][i];
		}
	}
	return max;
}

int select_action(int s, int a_num, float Qtable[num_s][num_a]){
	float max;
	int i = 0;
	int* i_max = new int[a_num];
	int num_i_max = 1;
	int a;

	i_max[0] = 0;
	max = Qtable[s][0];
	
	for(i=1;i<a_num;i++){
		if(Qtable[s][i]>max){
			max = Qtable[s][i];
			num_i_max = 1;
			i_max[0] = i;
		}
		else if(Qtable[s][i] == max){
			num_i_max++;
			i_max[num_i_max-1] = i;
		}
	}
	
	a = i_max[rand()%num_i_max];
	return a;
}


int main(int argc, char *argv[]){
	ros::init(argc, argv, "q_update_client");
	ros::NodeHandle n;
	
	FILE *fp;
	FILE *fpq;
	const char *fname = "/home/amsl/ros_catkin_ws/src/arm_q_learning/test_results/ql_test.txt";
	// const char *fqname = "q_table.txt";
	char fqname[100];
	
	fp = fopen(fname, "w");
	if(fp == NULL){
		printf("%sファイルが開けません\n", fname);
		return -1;
	}

	/*fpq = fopen(fqname, "w");
	if(fpq == NULL){
		printf("%sファイルが開けません\n", fqname);
		return -1;
	}*/
	
	ros::Subscriber sub_1 = n.subscribe("/infant/joint_states", 10, joint_states_callback);
	ros::Subscriber sub_2 = n.subscribe("/action_num", 10, action_num_callback);
	ros::Subscriber sub_3 = n.subscribe("/reward", 10, reward_callback);
	ros::Subscriber sub_4 = n.subscribe("/state_observation_flag", 10, state_observation_flag_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Int64>("/optimum_action_num", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Int64>("/num_state",1);
	ros::Publisher pub_4 = n.advertise<std_msgs::Int64>("/epsilon_greedy_flag", 1);
	ros::Publisher pub_5 = n.advertise<std_msgs::Int64>("/request_reward", 1);

	ros::ServiceClient client = n.serviceClient<joint_to_s::joint_state>("state_num");

	ros::Rate loop_rate(10);
	std_msgs::Int64 num_state;
	std_msgs::Int64 request_state_num;
	std_msgs::Int64 epsilon_greedy_flag;
	std_msgs::Int64 request_reward;
	
	int i, j, s, sd, step_count;
	int episode_count = 0;
	int count = 0;
	int temp_count = 0;
	float Qtable[num_s][num_a];
	float Qmax;

	
	s = 0;
	sd = 0;

	epsilon_greedy_flag.data = 0;
	request_reward.data = 0;

	joint_to_s::joint_state srv;
	

	int joint_state_int[3];
	
	for(i=0;i<num_s;i++){
		for(j=0;j<num_a;j++){
			Qtable[i][j] = 0.0;
			// printf("Q[%d][%d] = %lf\n", i, j, Qtable[i][j]);
		}
	}

	while(ros::ok()){
		if(wait_flag){
			printf("wait 3 seconds!!\n");
			count++;
			if(count == 30){
				wait_flag = false;
				select_action_pub_flag = false;
				q_update_flag = false;
 				state_observation_flag = true;
				count = 0;
				temp_count = 0;
 			}
			optimum_action.data = 0;
			num_state.data = 0;
			s = 0;
			reward = 0;
			pub_1.publish(optimum_action);
			pub_2.publish(num_state);
		}
		else{
			if(select_action_pub_flag){
				// optimum_action.data = select_action(s, num_a, Qtable);
				optimum_action.data = 9;
				pub_1.publish(optimum_action);
				select_action_pub_flag = false;
				if(temp_count >= 10){
					// state_observation_flag = false;
					select_action_pub_flag = false;
				}
			}
			
			if(state_observation_flag){
				srv.request.joint1 = joint_state[0];
				srv.request.joint3 = joint_state[1];
				srv.request.joint5 = joint_state[2];
			
				if(client.call(srv)){
					// ROS_INFO("state_num: %ld", srv.response.state);
					printf("\n");
					sd = srv.response.state;

					num_state.data = sd;
					pub_2.publish(num_state);
					cout<<"num_state :"<<num_state.data<<endl;
					printf("publish /num_state\n");

					pub_5.publish(request_reward);
					cout<<"reward : "<<reward<<endl;
					printf("request reward!!\n");

					// select_action_pub_flag = true;
					q_update_flag = true;
				}
				else{
					// ROS_ERROR("Failed to call service joint_to_s_server");
					return 1;
				}
				// if(temp_count == 30){
					// state_observation_flag = false;
				// }
				// else if(temp_count > 30){
					// state_observation_flag = false;
				// }	
				state_observation_flag = false;
			}

			if(q_update_flag){
				Qmax = max_Qval(sd, num_a, Qtable);
				Qtable[s][a] = (1- alpha) * Qtable[s][a] + alpha * ((float)reward + gamma_1 * Qmax);
				printf("Qtable update!\n");
				q_update_flag = false;
				select_action_pub_flag = true;
			}

			printf("\n");

			if(reward >= 5){
				printf("epsode : %d  ", episode_count);
				printf("step : %d  ", step_count);
				printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
				printf("TD error : %.3f ", reward + gamma_1 * Qmax -Qtable[s][a]);
				printf("reward : %.0f  ", reward);
				printf("success!!\n");

				fprintf(fp, "%d,%d\n", episode_count, step_count);
				printf("書き込み中\n");
				// Qtable[48999][9] = 2.0;

				/*for(i=0;i<num_s;i++){
					fprintf(fpq, "%d,%lf\n", i, Qtable[i][9]);
					printf("Q[%d][9] = %lf\n", i, Qtable[i][9]);
				}*/

				if(episode_count <= 1000){
					if((episode_count%50)==0){
						sprintf(fqname, "/home/amsl/ros_catkin_ws/src/arm_q_learning/q_table/q_table1_%d.txt", episode_count);
						fpq = fopen(fqname, "a");
						if(fpq == NULL){
							printf("%sファイルが開けません\n", fqname);
							return -1;
						}
						
						for(i=0;i<num_s;i++){
							printf("Qtable 書き込み\n");
							fprintf(fpq, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", Qtable[i][0], Qtable[i][1], Qtable[i][2], Qtable[i][3], Qtable[i][4], Qtable[i][5], Qtable[i][6], Qtable[i][7], Qtable[i][8], Qtable[i][9], Qtable[i][10], Qtable[i][11], Qtable[i][12], Qtable[i][13], Qtable[i][14], Qtable[i][15], Qtable[i][16], Qtable[i][17], Qtable[i][18], Qtable[i][19], Qtable[i][20], Qtable[i][21], Qtable[i][22], Qtable[i][23], Qtable[i][24], Qtable[i][25], Qtable[i][26]);
						}
						fclose(fpq);
					}
				}
			
				step_count = 0;
				episode_count++;
				
				optimum_action.data = 0;
				num_state.data = 0;
				pub_1.publish(optimum_action);
				pub_2.publish(num_state);

				wait_flag = true;
				
				// fclose(fpq);
				// fclose(fp);
				
			}
			else{	
				printf("epsode : %d  ", episode_count);
				printf(" step : %d  ", step_count);
				printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
				printf("TD error : %.3f ", reward + gamma_1 * Qmax -Qtable[s][a]);
				printf("reward : %.0f  ", reward);
				printf("\n");
				
				s = sd;
			}
			
			step_count++;
			temp_count++;

		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	// fclose(fp);
	// fclose(fpq);

	return 0;
}
