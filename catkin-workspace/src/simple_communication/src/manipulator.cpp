#include <stdint.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"


/**
 * Manipulator Node for Task 1
 */

void modifyCallback(const std_msgs::Int16 message)
{
  int16_t original = message.data;
  int32_t modified = original*original;

  ROS_INFO("Received: %d, Squared: %d", original, modified);
}


int main(int argc, char **argv)
{
  const char* node_name = "manipulator";
  const char* topic_name = "/task1/numbers";
  uint32_t queue_size = 200;

  ros::init(argc, argv, node_name);

  ros::NodeHandle node_handler;

  ros::Subscriber subscriber = \
                  node_handler.subscribe(topic_name, queue_size, modifyCallback);

  ros::spin();

  return 0;
}
