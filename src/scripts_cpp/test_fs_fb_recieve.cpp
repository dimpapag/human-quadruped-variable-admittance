#include "ros/ros.h"
#include "command_give/F_external.h"

void chatterCallback(const command_give::F_external::ConstPtr& msg)
{
  ROS_INFO("Received F_external:");
  for (size_t i = 0; i < msg->data.size(); ++i)
  {
    ROS_INFO(" %f", msg->data[i]);
  }

  // Extract F_s and F_b from the concatenated data
  std::vector<double> F_s(msg->data.begin(), msg->data.begin() + 6);
  std::vector<double> F_b(msg->data.begin() + 6, msg->data.end());

  ROS_INFO("F_s:");
  for (size_t i = 0; i < F_s.size(); ++i)
  {
    ROS_INFO(" %f", F_s[i]);
  }

  ROS_INFO("F_b:");
  for (size_t i = 0; i < F_b.size(); ++i)
  {
    ROS_INFO(" %f", F_b[i]);
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
