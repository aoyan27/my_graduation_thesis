#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <stdio.h>

using namespace std;

const int num_s = 70*70*10;
const int num_a = 3*3*3;

float alpha = 0.5;
float gamma_1 = 0.9;

int sd;
int a;

bool wait_flag = false;

bool pub_flag = false;

bool q_update_flag = false;

bool request_state_num_flag = true;

std_msgs::Int64 optimum_action;

float reward;

void state_num_callback(std_msgs::Int64 msg){
	sd = msg.data;
	// cout<<"state_num : "<<msg.data<<endl;
	pub_flag = true;
}

void action_num_callback(std_msgs::Int64 msg){
	a = msg.data;
	q_update_flag = true;
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
	ros::init(argc, argv, "q_update_csv");
	ros::NodeHandle n;
	
	FILE *fp;
	FILE *fpq;
	const char *fname = "ql_test.txt";
	const char *fqname = "q_table.txt";
	
	fp = fopen(fname, "a");
	if(fp == NULL){
		printf("%sファイルが開けません\n", fname);
		return -1;
	}

	fpq = fopen(fqname, "w");
	if(fpq == NULL){
		printf("%sファイルが開けません\n", fqname);
		return -1;
	}

	ros::Subscriber sub_1 = n.subscribe("/state_num", 10, state_num_callback);
	ros::Subscriber sub_2 = n.subscribe("/action_num", 10, action_num_callback);
	ros::Subscriber sub_3 = n.subscribe("/reward", 10, reward_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Int64>("/optimum_action_num", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Int64>("/num_state",1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Int64>("/request_state_num", 1);
	ros::Publisher pub_4 = n.advertise<std_msgs::Int64>("/epsilon_greedy_flag", 1);
	ros::Rate loop_rate(5);

	std_msgs::Int64 num_state;
	std_msgs::Int64 request_state_num;
	std_msgs::Int64 epsilon_greedy_flag;
	
	int i, j, s, step_count;
	int episode_count = 0;
	int count = 0;
	float Qtable[num_s][num_a];
	float Qmax;

	for(i=0;i<num_s;i++){
		for(j=0;j<num_a;j++){
			Qtable[i][j] = 0.0;
			// printf("Q[%d][%d] = %lf\n", i, j, Qtable[i][j]);
		}
	}
	
	s = 0;

	epsilon_greedy_flag.data = 0;

	while(ros::ok()){
		if(wait_flag){
			printf("wait 3 seconds!!\n");
			count++;
			if(count == 15){
				wait_flag = false;
				count = 0;
			}
			optimum_action.data = 0;
			num_state.data = 0;
			pub_1.publish(optimum_action);
			pub_2.publish(num_state);

		}
		else{
			if(request_state_num_flag){
				request_state_num.data = 0.0;
				pub_3.publish(request_state_num);
				// request_state_num_flag = false;
				// cout<<"request_state_num"<<endl;
			}
			
			// if(episode_count%10 == 0){
				// pub_4.publish(epsilon_greedy_flag);
				// q_update_flag = false;
				// cout<<"now evaluation!"<<endl;
			// }

			if(q_update_flag){
				num_state.data = s;
				// pub_2.publish(num_state);
		
				optimum_action.data = select_action(s, num_a, Qtable);
				// pub_1.publish(optimum_action);
				// cout<<"optimum action : "<<optimum_action.data<<endl;
				Qmax = max_Qval(sd, num_a, Qtable);
				Qtable[s][a] = (1 - alpha) * Qtable[s][a] + alpha *(reward + gamma_1 * Qmax);
				request_state_num_flag = true;
				q_update_flag = false;
				cout<<"Q update!"<<endl;
			}

			// cout<<"Qmax : "<<Qmax<<endl;
			// cout<<"reward :"<<reward<<endl;
			// cout<<Qtable[s][a]<<endl;		
			// printf("Q[%d][%d] = %f\n", s, a, Qtable[s][a]);
	
			// s = sd;
			// cout<<"state update"<<endl;
			cout<<endl;

			if(reward >= 5){
				// fp = fopen(fname, "a");
				// if(fp == NULL){
					// printf("%sファイルが開けません\n", fname);
					// return -1;
				// }

				printf("epsode : %d  ", episode_count);
				printf("step : %d  ", step_count);
				printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
				printf("TD error : %.3f ", reward + gamma_1 * Qmax -Qtable[s][a]);
				printf("reward : %.0f  ", reward);
				printf("success!!\n");

				fprintf(fp, "%d,%d\n", episode_count, step_count);
				printf("書き込み中\n");

				step_count = 0;
				episode_count++;
				
				optimum_action.data = 0;
				num_state.data = 0;
				if(pub_flag){
					pub_1.publish(optimum_action);
					pub_flag = false;
				}

				pub_2.publish(num_state);

				wait_flag = true;

				if((episode_count%10)==0){
					// cout<<"aaaaaaaaaaa"<<endl;
					for(i=0;i<num_s;i++){
						printf("Qtable 書き込み\n");
						fprintf(fpq, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n"
							, Qtable[i][0], Qtable[i][1], Qtable[i][2], Qtable[i][3], Qtable[i][4], Qtable[i][5], Qtable[i][6], Qtable[i][7], Qtable[i][8], Qtable[i][9], Qtable[i][10], Qtable[i][11], Qtable[i][12], Qtable[i][13], Qtable[i][14], Qtable[i][15], Qtable[i][16], Qtable[i][17], Qtable[i][18], Qtable[i][19], Qtable[i][20], Qtable[i][21], Qtable[i][22], Qtable[i][23], Qtable[i][24], Qtable[i][25], Qtable[i][26]);
					}
				}
				
				// fclose(fpq);
				// fclose(fp);
				
			}
			else{
				// if(episode_count%10 != 0){
					// printf("epsode : %d  ", episode_count);
					// printf(" step : %d  ", step_count);
					// printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
					// printf("TD error : %.3f ", reward + gamma_1 * Qmax -Qtable[s][a]);
					// printf("reward : %.0f  ", reward);
					// printf("\n");
				// }
				
				
				printf("epsode : %d  ", episode_count);
				printf(" step : %d  ", step_count);
				printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
				printf("TD error : %.3f ", reward + gamma_1 * Qmax -Qtable[s][a]);
				printf("reward : %.0f  ", reward);
				printf("\n");
				
				s = sd;

				if(pub_flag){
					pub_1.publish(optimum_action);
					pub_flag = false;
				}
				pub_2.publish(num_state);
				// cout<<"num_state : "<<num_state.data<<endl;
			}
			

			step_count++;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	fclose(fpq);
	fclose(fp);

	return 0;
}
