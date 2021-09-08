#ifndef PUBLISHER_SUBSCRIBER_H
#define PUBLISHER_SUBSCRIBER_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32.h>

template<typename PublishT, typename SubscribeT>
class PublisherSubscriber
{
public: 
	PublisherSubscriber() {}
	// overloaded constructor
	PublisherSubscriber(std::string publishTopicName, std::string subscribeTopicName, int queueSize)
	{
		publisherObject = nH.advertise<PublishT>(publishTopicName, queueSize);
		subscriberObject = nH.subscribe<SubscribeT>(subscribeTopicName, queueSize, &PublisherSubscriber::subscriberCallback, this);
	}
	void subscriberCallback(const typename SubscribeT::ConstPtr& receiveMsg);

protected:
	// attributes of this class
	ros::Subscriber subscriberObject;
	ros::Publisher publisherObject;
	ros::NodeHandle nH;
};

#endif 
