#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv){
  ros::init(argc, argv, "simulator");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("/simulator_test", 1);

  ros::Rate loop_rate(10);

  std_msgs::Bool msg;
  msg.data = false;

  int count = 0;
  while (ros::ok()){

    if (count % 50 == 0){
        msg.data = !msg.data;
    }

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  return 0;
}