#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub =
      nh.advertise<std_msgs::String>("chatter", 10);

  ros::Rate loop_rate(10);
  int count = 0;

  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Hello ROS from talker: " << count;
    msg.data = ss.str();

    ROS_INFO_STREAM("Publishing: " << msg.data);
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}

