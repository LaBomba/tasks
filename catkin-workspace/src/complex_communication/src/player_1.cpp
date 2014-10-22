#include "ros/ros.h"
#include "complex_communication/Turn.h"

void chatterCallback(complex_communication::Turn msg)
{
  ROS_INFO("I heard: [%d]", msg.spot);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle node_hanlder;

  ros::Subscriber sub = node_hanlder.subscribe("/task2/table", 200, chatterCallback);

  ros::spin();

}
