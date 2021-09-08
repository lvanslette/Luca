#include <ros/ros.h>
#include "include/PublisherSubscriber.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

void multiply(float mat1[4][3], float mat2[3][1], float res[4]) {
    for (int i=0; i<4; i++){
        float temp = 0;
        for (int j=0; j<3; j++) {
           temp += mat1[i][j]*mat2[j][0]; 
        }
        res[i] = temp;
    }

}

template<>
void PublisherSubscriber<std_msgs::Float32MultiArray, std_msgs::Int32MultiArray>::subscriberCallback(const
std_msgs::Int32MultiArray::ConstPtr& receivedMsg)
{
	int distance[4];
    // distance array: [front, back, left, right]
    // ex: distance[0] == 1 means there is something in front of us
    distance[0] = receivedMsg->data[0];
    distance[1] = receivedMsg->data[1];
    distance[2] = receivedMsg->data[2];
    distance[3] = receivedMsg->data[3];
    
    // default forward velocity = 0.25m/s
	float x_velocity = 0.0;
	float y_velocity = 0.0;
	float angular_velocity = 0.0;
    // adjust velocity based on IR sensor measurements: distance[i] == 1 means there is something close to the robot
    float default_velocity = 0.1;
    x_velocity = -default_velocity*distance[0] + default_velocity*distance[1];  
    // ex: distance[3] == 1 means theres something on the left of us, we want to go in the +y direction (to the right)
    y_velocity = -default_velocity*distance[3] + default_velocity*distance[2];  
    	
    ROS_INFO("x: %.2f, y: %.2f, phi: %.2f", x_velocity, y_velocity, angular_velocity);

    // calculate the wheel velocities
    float velocities[3][1] = { { angular_velocity },
                                { x_velocity },
                                { y_velocity } };
    float H_0[4][3] = { { -10.0, 1.0, -1.0 },
                        { 10.0, 1.0, 1.0 },
                        { -10.0, 1.0, 1.0 },
                        { 10.0, 1.0, -1.0 } };
    float u[4];

    multiply(H_0, velocities, u);

    ROS_INFO("u1: %.5f, u2: %.5f, u3: %.5f, u4: %.5f", u[0], u[1], u[2], u[3]);

	std_msgs::Float32MultiArray u_w;
	u_w.data.resize(4);
	u_w.data[0] = u[0];
	u_w.data[1] = u[1];
	u_w.data[2] = u[2];
	u_w.data[3] = u[3];
	
	publisherObject.publish(u_w);
}

int main(int argc, char **argv)
{
	// set up ROS
	ros::init(argc, argv, "motion_logic");
	// publish type float[], subscribe type string
	PublisherSubscriber<std_msgs::Float32MultiArray, std_msgs::Int32MultiArray> parrot("echo", "measurement", 1);
	ros::spin();
}
