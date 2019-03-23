/*
*USING ODOMETRY
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "kobuki_msgs/BumperEvent.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

#include "sstream"
#include "math.h"

class TurtleBot_Pattern {

private:
 bool run;
 bool hit;
 int continuous_hit;
 int turn_counter;

 ros::NodeHandle node;
 ros::Publisher wheelPub;
 ros::Subscriber bumperSub;
 geometry_msgs::Pose2D current_pose;
 ros::Publisher pub_pose2d;
 ros::Subscriber sub_odometry;

 ros::Time start_time;

 geometry_msgs::Twist base_cmd_velocity;

 void move(double distance, int direction);
 void bumperHandler(const kobuki_msgs::BumperEventConstPtr bumperMsg);
 void odomCallback(const nav_msgs::OdometryConstPtr& msg);
 void setVelocity (double linear, double angular);
 void pause();
 void turnRight();
 void turnLeft();

public:
 TurtleBot_Pattern();
 void runPattern();

};

 TurtleBot_Pattern::TurtleBot_Pattern() {
   run = true;
   hit = false;
   continuous_hit = 0;
   turn_counter = 0;

   //  "/pattern/cmd_vel"
   wheelPub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 2);
   bumperSub = node.subscribe("/mobile_base/events/bumper", 1, &TurtleBot_Pattern::bumperHandler, this);
   sub_odometry = node.subscribe("odom", 1, &TurtleBot_Pattern::odomCallback, this);
   pub_pose2d = node.advertise<geometry_msgs::Pose2D>("/pose2d", 1);
 }

 void TurtleBot_Pattern::runPattern() {
    // ros::Rate rate(10);   

    // ROS_INFO("move forward");
    // ros::Time start = ros::Time::now();

    // while(ros::ok() && current_pose.x < 2.5) 
    // { 
    //     ROS_INFO("Curretnt X:");
    //     ROS_INFO_STREAM(current_pose.x);
    //     ROS_INFO("Curretnt Y:");
    //     ROS_INFO_STREAM(current_pose.y);

    //     geometry_msgs::Twist move; //velocity controls 
    //     move.linear.x = 0.1; //speed value m/s 
    //     move.angular.z = 0; wheelPub.publish(move); 
    //     ros::spinOnce(); rate.sleep(); 
    // } 
    // //turn right 
    // ROS_INFO("turn right"); 
    // ros::Time start_turn = ros::Time::now(); 
    
    // while(ros::ok() && current_pose.theta > -M_PI/2)
    // {
    //     ROS_INFO("Curretnt X:");
    //     ROS_INFO_STREAM(current_pose.x);
    //     ROS_INFO("Curretnt Y:");
    //     ROS_INFO_STREAM(current_pose.y);

    //     geometry_msgs::Twist move;
    //     //velocity controls
    //     move.linear.x = 0; //speed value m/s
    //     move.angular.z = -0.1;
    //     wheelPub.publish(move);
    
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // //move forward again
    // ROS_INFO("move forward");
    // ros::Time start2 = ros::Time::now();
    
    // while(ros::ok() && current_pose.y > -2.5)
    // {
    //     ROS_INFO("Curretnt X:");
    //     ROS_INFO_STREAM(current_pose.x);
    //     ROS_INFO("Curretnt Y:");
    //     ROS_INFO_STREAM(current_pose.y);

    //     geometry_msgs::Twist move;
    //     //velocity controls
    //     move.linear.x = 0.1; //speed value m/s
    //     move.angular.z = 0;
    //     wheelPub.publish(move);
    
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    
    // // just stop
    // while(ros::ok()) {
    //     geometry_msgs::Twist move;
    //     move.linear.x = 0;
    //     move.angular.z = 0;
    //     wheelPub.publish(move);
    
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    while(run && !hit) {
 
        double distance = 2;

        ROS_INFO("Curretnt X:");
        ROS_INFO_STREAM(current_pose.x);
        ROS_INFO("Curretnt Y:");
        ROS_INFO_STREAM(current_pose.y);

        ROS_INFO("Go Straight");
        move(distance, 1);

        ROS_INFO("Turn Right");
        turnRight();

        ROS_INFO("Go Straight");
        move(distance, 4);

        ROS_INFO("Turn Right");
        turnRight();

        ROS_INFO("Go Straight");
        move(distance, 2);

        ROS_INFO("Turn Right");
        turnRight();

        distance -= 0.2;

        ROS_INFO("Go Straight");
        move(distance, 3);

        distance -= 0.2;

        if (distance <= 0)
        hit = true;

    }
 }


/*
*   1 - Right
*   2 - Left
*   3 - Up
*   4 - Down
*/
 void TurtleBot_Pattern::move(double distance, int direction) {
     ros::Rate loop_rate(10);
     setVelocity(0.1, 0.0);

     ROS_INFO("Curretnt X:");
     ROS_INFO_STREAM(current_pose.x);
     ROS_INFO("Curretnt Y:");
     ROS_INFO_STREAM(current_pose.y);

     switch (direction) {
       case 1:
         distance = current_pose.x + distance;
         ROS_INFO("X");
         ROS_INFO_STREAM(distance);
         while(ros::ok() && current_pose.x < distance)
         {
            ROS_INFO("Curretnt X:");
            ROS_INFO_STREAM(current_pose.x);
            ROS_INFO("Curretnt Y:");
            ROS_INFO_STREAM(current_pose.y);

             wheelPub.publish(base_cmd_velocity);
             ros::spinOnce();
             loop_rate.sleep();
         }
         break;
       case 2:
         distance = current_pose.x - distance;
         ROS_INFO("X");
         ROS_INFO_STREAM(distance);
         while(ros::ok() && current_pose.x > distance)
         {
            ROS_INFO("Curretnt X:");
            ROS_INFO_STREAM(current_pose.x);
            ROS_INFO("Curretnt Y:");
            ROS_INFO_STREAM(current_pose.y);

             wheelPub.publish(base_cmd_velocity);
             ros::spinOnce();
             loop_rate.sleep();
         }
         break;
       case 3:
         distance = current_pose.y + distance;
         ROS_INFO("Y");
         ROS_INFO_STREAM(distance);
         while(ros::ok() && current_pose.y < distance)
         {
            ROS_INFO("Curretnt X:");
            ROS_INFO_STREAM(current_pose.x);
            ROS_INFO("Curretnt Y:");
            ROS_INFO_STREAM(current_pose.y);
            
             wheelPub.publish(base_cmd_velocity);
             ros::spinOnce();
             loop_rate.sleep();
         }
         break;
       case 4:
         distance = current_pose.y - distance;
         ROS_INFO("Y");
         ROS_INFO_STREAM(distance);
         while(ros::ok() && current_pose.y > distance)
         {
            ROS_INFO("Curretnt X:");
            ROS_INFO_STREAM(current_pose.x);
            ROS_INFO("Curretnt Y:");
            ROS_INFO_STREAM(current_pose.y);

             wheelPub.publish(base_cmd_velocity);
             ros::spinOnce();
             loop_rate.sleep();
         }
         break;
       default:
         break;
     }
 }

 void TurtleBot_Pattern::bumperHandler(const kobuki_msgs::BumperEventConstPtr bumperMsg) {
   if (bumperMsg->state == kobuki_msgs::BumperEvent::PRESSED)
   {
         ros::Rate loop_rate(10);
         hit = true;
         setVelocity(0.0, 0.0);
         wheelPub.publish(base_cmd_velocity);
         ros::spinOnce();
         loop_rate.sleep();
         continuous_hit++;
         ROS_INFO("BumperPressed");
         if (continuous_hit >= 3){
             run = false;
         }
   }
 }

 void TurtleBot_Pattern::odomCallback(const nav_msgs::OdometryConstPtr& msg)
 {
     // linear position
     current_pose.x = msg->pose.pose.position.x;
     current_pose.y = msg->pose.pose.position.y;
    
     // quaternion to RPY conversion
     tf::Quaternion q(
         msg->pose.pose.orientation.x,
         msg->pose.pose.orientation.y,
         msg->pose.pose.orientation.z,
         msg->pose.pose.orientation.w);
     tf::Matrix3x3 m(q);
     double roll, pitch, yaw;
     m.getRPY(roll, pitch, yaw);
    
     // angular position
     current_pose.theta = yaw;
     pub_pose2d.publish(current_pose);
 }

 void TurtleBot_Pattern::setVelocity (double linear, double angular) {
   base_cmd_velocity.linear.x = linear;
   base_cmd_velocity.angular.z = angular;
 }

 void TurtleBot_Pattern::pause() {
   setVelocity(0.0, 0.0);
   ros::Rate loop_rate(10);
   start_time = ros::Time::now();
              
   while (ros::Time::now() - start_time < ros::Duration(0.5)){
       wheelPub.publish(base_cmd_velocity);
       loop_rate.sleep();
   }
 }

 void TurtleBot_Pattern::turnRight() {
   ros::Rate loop_rate(10);
   setVelocity(0.0, -0.3);

   while(ros::ok() && current_pose.theta > -M_PI/2)
   {
       wheelPub.publish(base_cmd_velocity);   
       ros::spinOnce();
       loop_rate.sleep();
   }
 }

 void TurtleBot_Pattern::turnLeft() {
   ros::Rate loop_rate(10);
   setVelocity(0.0, 0.3);

   while(ros::ok() && current_pose.theta > M_PI/2)
   {
       wheelPub.publish(base_cmd_velocity);   
       ros::spinOnce();
       loop_rate.sleep();
   }
 }


int main(int argc, char **argv) {
 ros::init(argc, argv, "Pattern");
 TurtleBot_Pattern turtleBot;

 ROS_INFO("Pattern Executing");

 if (ros::ok()) {
     turtleBot.runPattern();
 }

 return 0;
}
