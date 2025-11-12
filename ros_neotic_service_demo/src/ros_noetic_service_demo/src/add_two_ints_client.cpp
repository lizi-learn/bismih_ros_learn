#include <cstdlib>

#include "ros/ros.h"
#include "ros_noetic_service_demo/AddTwoInts.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_ERROR("Usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<ros_noetic_service_demo::AddTwoInts>("add_two_ints");

  ros_noetic_service_demo::AddTwoInts srv;
  srv.request.a = std::atoll(argv[1]);
  srv.request.b = std::atoll(argv[2]);

  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", static_cast<long>(srv.response.sum));
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}

