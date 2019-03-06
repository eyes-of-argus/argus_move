#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include "kobuki_msgs/BumperEvent.h"

#include <sstream>

bool run = true;
bool hit = false;
int continuous_hit = 0;
int turn_counter = 0;


void bumperEventHandler(const kobuki_msgs::BumperEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
  {
    hit = true;
        
    continuous_hit++;
    ROS_INFO("BumperPressed");
    if (continuous_hit >= 3){
        run = false;
    }
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle wheel;
  ros::Publisher chatter_pub = wheel.advertise<geometry_msgs::Twist>("/pattern/cmd_vel", 2);
 
  ros::NodeHandle bumper;
  ros::Subscriber bumper_sub = bumper.subscribe("/mobile_base/events/bumper", 1, bumperEventHandler);

  ros::Rate loop_rate(10);


  ROS_INFO("Starting to move forward");
    
  geometry_msgs::Twist msg;
  msg.linear.x = 0.5;
  msg.angular.z = 0.0;
 
  geometry_msgs::Twist base_cmd_turn_right;
  base_cmd_turn_right.linear.x = 0;
  base_cmd_turn_right.angular.z = -0.415;
 
  ros::Time start_time = ros::Time::now();
  ros::Duration run_timeout(5.0);
  ros::Duration turn_timeout(3.7);

  while (ros::ok() && run)
  {

    if(!hit){
        chatter_pub.publish(msg);
        loop_rate.sleep();
    
        if (ros::Time::now() - start_time >= run_timeout){
            start_time = ros::Time::now();
            
            while (ros::Time::now() - start_time < turn_timeout){
        chatter_pub.publish(base_cmd_turn_right);
        loop_rate.sleep();
        }
        
        turn_counter++;
            start_time = ros::Time::now();
        }
    
    if (turn_counter >= 3) {
        turn_counter = -1;
        run_timeout = run_timeout - ros::Duration(1.0);
    }

    if (run_timeout == ros::Duration(0.0))
        run = false;

    continuous_hit = 0;
    }
    else{
    start_time = ros::Time::now();
        while (ros::Time::now() - start_time < turn_timeout){
        chatter_pub.publish(base_cmd_turn_right);
        loop_rate.sleep();
        }

        start_time = ros::Time::now();
        
        hit = false;
    }

    ros::spinOnce();
  }

  return 0;
}

