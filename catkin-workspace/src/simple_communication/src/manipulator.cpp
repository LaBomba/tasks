#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

#include <stdint.h>

/**
 * Manipulator Node for Task 1
 */

void random_number_callback(const std_msgs::Int16 message)
{
    int16_t original = message.data;
    int32_t modified = original*original;
    ROS_INFO("Received: %d, Squared: %d", original, modified);
}


int main(int argc, char **argv)
{
    const char *node_name = "manipulator";
    const char *topic_name = "/test1/numbers";
    uint32_t queue_size = 200;

    ros::init(argc, argv, node_name);

    ros::NodeHandle node_handler;

    ros::Subscriber subscriber = node_handler.subscribe(topic_name, queue_size, random_number_callback);

    ros::spin();

    return 0;
}
