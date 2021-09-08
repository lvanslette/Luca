#include "ros/ros.h"
#include "std_msgs/Float32.h"

// we will be receiving messages from a ROS topic in this program
//


void chatterCallback(const std_msgs::Float32::ConstPtr& msg)
{
	ROS_INFO("I heard: [%.2f]", msg->data);
}

int main(int argc, char **argv)
{
	// initialize ros, same as talker
	ros::init(argc, argv, "listener");
	// initialize node, same as talker
	ros::NodeHandle n;

	// subscribe to topic chatter
	ros::Subscriber sub = n.subscribe("measurement", 1000, chatterCallback);
		// messages on the topic are passed to the callback function, in this case chatterCallback,
		// the second parameter is the size of the message queue, if messages are arriving too fast then
		// 	the newest 1000 messages will be kept in a buffer

	// enters a loop, calling message callbacks as fast as possible. will exit only when Ctrl-C is pressed, or node is shut down by master
	ros::spin();

	return 0;

}
