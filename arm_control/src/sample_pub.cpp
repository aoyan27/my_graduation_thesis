#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "sample_pub");
	ros::NodeHandle n;

	ros::Publisher pub_1 = n.advertise<std_msgs::Float64>("/joint1_pose_com", 1);
	ros::Publisher pub_2 = n.advertise<std_msgs::Float64>("/joint3_pose_com", 1);
	ros::Publisher pub_3 = n.advertise<std_msgs::Float64>("/joint5_pose_com", 1);

	ros::Publisher pub_4 = n.advertise<std_msgs::Float64>("/infant/joint6_position_controller/command", 1);
	ros::Publisher pub_5 = n.advertise<std_msgs::Float64>("/infant/joint7_position_controller/command", 1);

	int count = 0;
	int state = 0;
	ros::Rate loop_rate(10);
	std_msgs::Float64 joint6, joint7;

	joint6.data = -0.05;
	joint7.data = 0.05;

	while(ros::ok()){
		std_msgs::Float64 msg1, msg2, msg3;
		msg1.data = 0.0;
		msg2.data = 95.0;
		msg3.data = 45.0;
		if(count == 10){
			cout<<"publish"<<endl;
			pub_1.publish(msg1);
			pub_2.publish(msg2);
			pub_3.publish(msg3);

			pub_4.publish(joint6);
			pub_5.publish(joint7);
			state = 700 * (msg1.data + 35) + 10 * (msg2.data - 40) + 1 * (msg3.data - 30);
			printf("state : %d\n", state);
			printf("joint1 : %.0f\n", msg1.data);
			printf("joint3 : %.0f\n", msg2.data);
			printf("joint5 : %.0f\n", msg3.data);
			count = 0;
		}

		count++;

		loop_rate.sleep();
	}
	return 0;
}
