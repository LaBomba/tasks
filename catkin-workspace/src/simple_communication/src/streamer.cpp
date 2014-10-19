#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

#include <stdint.h>
#include <cstdlib>


/**
 * Streamer Node for Task 1
 */

int main(int argc, char **argv)
{
    const char *node_name = "streamer";
    const char *topic_name = "/test1/numbers";
    uint32_t queue_size = 200;

    // Advertise one message every second
    float publish_rate = 1;

    ros::init(argc, argv, node_name);

    ros::NodeHandle node_handler;

    ros::Publisher node_publisher = node_handler.advertise<std_msgs::Int16>(topic_name, queue_size);

    ros::Rate loop_rate(publish_rate);

    while (ros::ok())
    {
        std_msgs::Int16 message;

        message.data = rand();

        ROS_INFO("%s said: %d", node_name, message.data);

        node_publisher.publish(message);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;

}
