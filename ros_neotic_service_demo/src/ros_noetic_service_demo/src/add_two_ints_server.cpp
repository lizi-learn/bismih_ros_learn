#include "ros/ros.h"
#include "ros_noetic_service_demo/AddTwoInts.h"

bool add(ros_noetic_service_demo::AddTwoInts::Request &req,
         ros_noetic_service_demo::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("Request: a=%ld b=%ld", static_cast<long>(req.a), static_cast<long>(req.b));
  ROS_INFO("Responding with sum=%ld", static_cast<long>(res.sum));
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}

