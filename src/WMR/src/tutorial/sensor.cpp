#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include <wiringPi.h>

float read_sensor(void) 
{
	return 10.5;
}	
	
// all this is doing is sending a message over ROS
int main(int argc, char **argv)
{
	// initialize GPIO pins
	const int TRIGGER_PIN = 18;
	const int ECHO_PIN = 5;
	wiringPiSetup();

	// initialize ROS
	ros::init(argc, argv, "talker");	// use argv, argc for 'name remapping'
						// -> name remapping: any ROS name w/in a node can be remapped when launched at the command line
						// 	lets you launch the same node under multiple configurations 
	// initialize Node
	ros::NodeHandle n;	// use n to communicate with other Nodes

	// create an object that is used to publish messages to the topic 'chatter'
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("measurement", 1000);
		// advertise() function: create a topic to 'advertise' to. we can now publish() to the topic 'chatter'
		// 	the second argument says how many messages to buffer before throwing some away (size of publishing queue)
	
	// set ros loop rate at 10 Hz
	ros::Rate loop_rate(10);
		// ros::Rate object allows to specify a frequency we should loop at 
		// 	used with Rate::sleep()
	
	// count of how many messages we have sent (used to create a unique string for each message)
	int count = 0;
	// while Ctrl-C hasn't been pressed, or kicked off network, or ros::shutdown() hasnt been called, or ros::NodeHandles haven't been destroyed
	while (ros::ok())
	{
		// create message using String datatype
		std_msgs::Float32 msg;

		/* std::stringstream ss;
		ss << "hello world " << count;
		// add the string to the msg object
		msg.data = ss.str();
		*/
		msg.data = read_sensor();
		// replacement for printf/cout
		ROS_INFO("%.2f", msg.data);

		// publish message with datatype String to the topic "chatter"
		chatter_pub.publish(msg);

		// used for receiving callbacks (when we are subscribed to a topic as well as publishing to "chatter")
		ros::spinOnce();

		// use this to make sure we hit out 10 Hz publishing rate (publish 10 times a second)
		loop_rate.sleep();
		++count;
	}

	return 0;
}
	 


