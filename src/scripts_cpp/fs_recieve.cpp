#include "ros/ros.h"
#include "command_give/F_external.h"

void chatterCallback(const command_give::F_external::ConstPtr& msg)
{
  ROS_INFO("Received F_external:");
  for (size_t i = 0; i < msg->data.size(); ++i)
  {
    ROS_INFO(" %f", msg->data[i]);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("F_external", 10, chatterCallback);
  ros::spin();
  return 0;
}
