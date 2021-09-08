#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <stdio.h>

void binary(int count, int res[4]) {
    int i = 0;
    res[0] = 0;
    res[1] = 0;
    res[2] = 0;
    res[3] = 0;
    while (count > 0) {
        res[i] = count%2;
        count = count/2;
        i++;
    }
}
void single_sensor(int count, int res[4]) {
    int singles[4][4] = { { 1, 0, 1, 0 },
                          { 0, 1, 0, 1 },
                          { 1, 0, 0, 1 },
                          { 0, 1, 1, 0 } };
    int index = count % 4;
    res[0] = singles[index][0];
    res[1] = singles[index][1];
    res[2] = singles[index][2];
    res[3] = singles[index][3];

}
// all this is doing is sending a message over ROS
int main(int argc, char **argv)
{
	// initialize ROS
	ros::init(argc, argv, "talker");	// use argv, argc for 'name remapping'
						// -> name remapping: any ROS name w/in a node can be remapped when launched at the command line
						// 	lets you launch the same node under multiple configurations 
	// initialize Node
	ros::NodeHandle n;	// use n to communicate with other Nodes

	// create an object that is used to publish messages to the topic 'chatter'
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("measurement", 1000);
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
        sleep(1);
		int binary_num[4];
        //binary(count, binary_num);
        single_sensor(count, binary_num); 
        ROS_INFO("%d, %d, %d, %d", binary_num[3], binary_num[2], binary_num[1], binary_num[0]);

        std_msgs::Int32MultiArray measurement;
        measurement.data.resize(4);
        measurement.data[0] = binary_num[0];
        measurement.data[1] = binary_num[1];
        measurement.data[2] = binary_num[2];
        measurement.data[3] = binary_num[3];

		// replacement for printf/cout
		ROS_INFO("%d", count);

		// publish message with datatype String to the topic "chatter"
		chatter_pub.publish(measurement);

		// used for receiving callbacks (when we are subscribed to a topic as well as publishing to "chatter")
		ros::spinOnce();

		// use this to make sure we hit out 10 Hz publishing rate (publish 10 times a second)
		loop_rate.sleep();
		++count;
        if (count == 16) {
            count = 0;
        }
	}

	return 0;
}

		 


