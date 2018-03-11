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

std_msgs::Int64 optimum_action;

float reward;

void state_num_callback(std_msgs::Int64 msg){
	sd = msg.data;
	// cout<<"msg : "<<msg.data<<endl;
}

void action_num_callback(std_msgs::Int64 msg){
	a = msg.data;
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
	ros::init(argc, argv, "q_update");
	ros::NodeHandle n;

	ros::Subscriber sub_1 = n.subscribe("/state_num", 10, state_num_callback);
	ros::Subscriber sub_2 = n.subscribe("/action_num", 10, action_num_callback);
	ros::Subscriber sub_3 = n.subscribe("/reward", 10, reward_callback);

	ros::Publisher pub_1 = n.advertise<std_msgs::Int64>("/optimum_action_num", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Int64>("/num_state",1);

	ros::Rate loop_rate(5);

	std_msgs::Int64 num_state;
	
	int i, j, s, step_count;
	int episode_count = 0;
	int count = 0;
	float Qtable[num_s][num_a];
	float Qmax;

	for(i=0;i<num_s;i++){
		for(j=0;j<num_a;j++){
			Qtable[i][j] = 0.0;
			printf("Q[%d][%d] = %lf\n", i, j, Qtable[i][j]);
		}
	}
	
	s = 0;

	while(ros::ok()){
		if(wait_flag){
			printf("wait 3 seconds!!\n");
			count++;
			if(count == 15){
				wait_flag = false;
				count = 0;
			}
		}
		else{
			num_state.data = s;
			// pub_2.publish(num_state);
	
			optimum_action.data = select_action(s, num_a, Qtable);
			// pub_1.publish(optimum_action);
			// cout<<"optimum action : "<<optimum_action.data<<endl;
			Qmax = max_Qval(sd, num_a, Qtable);
			Qtable[s][a] = (1 - alpha) * Qtable[s][a] + alpha *(reward + gamma_1 * Qmax);
		
			// cout<<"Qmax : "<<Qmax<<endl;
			// cout<<"reward :"<<reward<<endl;
			// cout<<Qtable[s][a]<<endl;		
			// printf("Q[%d][%d] = %f\n", s, a, Qtable[s][a]);
	
			// s = sd;
			// cout<<"state update"<<endl;
			cout<<endl;

			if(reward >= 4){
				printf("epsode : %d  ", episode_count);
				printf("step : %d  ", step_count);
				printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
				printf("reward : %.0f  ", reward);
				printf("success!!\n");
				step_count = 0;
				episode_count++;
				
				optimum_action.data = 0;	
				pub_1.publish(optimum_action);
				num_state.data = 0;
				pub_2.publish(num_state);

				wait_flag = true;
			}
			else{
				printf("epsode : %d  ", episode_count);
				printf(" step : %d  ", step_count);
				printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
				printf("reward : %.0f  ", reward);
				printf("\n");
				
				s = sd;
				
				pub_1.publish(optimum_action);
				pub_2.publish(num_state);
			}
			
	
	
		/*	if(step_count < 200){
				if(reward >= 4){
					printf("epsode : %d  ", episode_count);
					printf("step : %d  ", step_count);
					printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
					printf("reward : %.0f  ", reward);
					printf("success!!\n");
					step_count = 0;
					episode_count++;
					
					optimum_action.data = 0;	
					pub_1.publish(optimum_action);
					num_state.data = 0;
					pub_2.publish(num_state);

					wait_flag = true;
				}
				else{
					printf("epsode : %d  ", episode_count);
					printf(" step : %d  ", step_count);
					printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
					printf("reward : %.0f  ", reward);
					printf("\n");
					
					s = sd;
					
					pub_1.publish(optimum_action);
					pub_2.publish(num_state);
				}
			}
			else{
				printf("episode : %d  ", episode_count);
				printf("step : %d  ", step_count);
				printf("Q[%d][%d] = %.3f  ", s, a, Qtable[s][a]);
				printf("reward : %.0f  ", reward);
				printf("failure!!\n");
				step_count = 0;
				episode_count++;
	
				optimum_action.data = 0;
				pub_1.publish(optimum_action);
				num_state.data = 0;
				pub_2.publish(num_state);
				
				wait_flag = true;
			}*/

			step_count++;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
