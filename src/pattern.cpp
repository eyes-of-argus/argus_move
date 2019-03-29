//Inspired by https://github.com/aniskoubaa/gaitech_edu/blob/master/src/turtlebot/navigation/free_space_navigation/free_space_navigation.cpp

/*
*USING ODOMETRY
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include "kobuki_msgs/BumperEvent.h"

// #include <fstream>
// #include "std_msgs/String.h"
#include "math.h"


using namespace std;

#define LINEAR_VELOCITY_MINIMUM_THRESHOLD 0.2
#define ANGULAR_VELOCITY_MINIMUM_THRESHOLD 0.4

class TurtleBot_Pattern {

   private:
       bool run;
       bool hit;
       // int continuous_hit;
       // int turn_counter;
       double distance;

       ros::NodeHandle node;
       ros::Publisher wheel_pub;
       ros::Subscriber bumper_sub;
       //declare subscribers
       ros::Subscriber pose_sub;
       //global variable to update the position of the robot
       nav_msgs::Odometry turtlebot_odom_pose;
       geometry_msgs::Twist VelocityMessage;

       //callback function for the /odom topic to update the pose
       void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message);
       //callback function for the //mobile_base/events/bumper
       void bumperHandler(const kobuki_msgs::BumperEventConstPtr bumperMsg);
       //the function that makes the robot moves forward and backward
       void move_v1(double speed, double distance, bool isForward);
      
       //the function that makes the robot rotates left and right
       double rotate(double ang_vel, double angle_radian, bool isClockwise);

       double degree2radian(double degreeAngle);
       double radian2degree(double radianAngle);   
      

   public:
       TurtleBot_Pattern();
       void movePattern (double distance, double moving_speed, double turning_speed);

};


TurtleBot_Pattern::TurtleBot_Pattern() {
   run = true;
   hit = false;

   //  "/pattern/cmd_vel"
   wheel_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 2); //"/mobile_base/commands/velocity"
   bumper_sub = node.subscribe("/mobile_base/events/bumper", 1, &TurtleBot_Pattern::bumperHandler, this);
   pose_sub = node.subscribe("odom", 1, &TurtleBot_Pattern::poseCallback, this);
}


void TurtleBot_Pattern::bumperHandler(const kobuki_msgs::BumperEventConstPtr bumperMsg) {
   if (bumperMsg->state == kobuki_msgs::BumperEvent::PRESSED)
   {
       ros::Rate loop(10);
       VelocityMessage.linear.x = VelocityMessage.linear.y = VelocityMessage.angular.z = 0;
       ROS_INFO("BumperPressed");
       ros::Time starting_time = ros::Time::now();
       ros::Duration pause_time = ros::Duration(2);

       while (ros::Time::now() - starting_time < pause_time){
           wheel_pub.publish(VelocityMessage);
           ros::spinOnce();
           loop.sleep();
       }
              
   }
}


void TurtleBot_Pattern::poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
   turtlebot_odom_pose.pose.pose.position.x = pose_message->pose.pose.position.x;
   turtlebot_odom_pose.pose.pose.position.y = pose_message->pose.pose.position.y;
   turtlebot_odom_pose.pose.pose.position.z = pose_message->pose.pose.position.z;

   turtlebot_odom_pose.pose.pose.orientation.w = pose_message->pose.pose.orientation.w;
   turtlebot_odom_pose.pose.pose.orientation.x = pose_message->pose.pose.orientation.x;
   turtlebot_odom_pose.pose.pose.orientation.y = pose_message->pose.pose.orientation.y;
   turtlebot_odom_pose.pose.pose.orientation.z = pose_message->pose.pose.orientation.z;
}


void TurtleBot_Pattern::movePattern(double distance, double moving_speed, double turning_speed){
   int trigger = 2;
  
   ros::Rate loop(10);
   ros::spinOnce();

   loop.sleep();

   while (distance > 0) {
       for (int i = 0; i < 4 && distance > 0; i++){
           move_v1(moving_speed, distance, true);
           rotate (turning_speed, degree2radian(90), true);
           if (i == trigger){
               distance -= 0.5;
               if (trigger == 2) {
                   trigger = 0;
               }
               else{
                   trigger = 2;
               }
              
           }
       }
   }
}


/**
* a function that makes the robot move straight
* @param speed: represents the speed of the robot the robot
* @param distance: represents the distance to move by the robot
* @param isForward: if true, the robot moves forward,otherwise, it moves backward
*
* Method 1: using tf and Calculate the distance between the two transformations
*/
void TurtleBot_Pattern::move_v1(double speed, double distance, bool isForward){
   //declare tf transform listener: this transform listener will be used to listen and capture the transformation between
   // the /odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
   tf::TransformListener listener;
   //declare tf transform
   //init_transform: is the transformation before starting the motion
   tf::StampedTransform init_transform;
   //current_transformation: is the transformation while the robot is moving
   tf::StampedTransform current_transform;


   //set the linear velocity to a positive value if isFoward is true
   if (isForward)
       VelocityMessage.linear.x = abs(speed);
   else //else set the velocity to negative value to move backward
       VelocityMessage.linear.x = -abs(speed);

   //all velocities of other axes must be zero.
   VelocityMessage.linear.y = 0;
   VelocityMessage.linear.z = 0;
   //The angular velocity of all axes must be zero because we want  a straight motion
   VelocityMessage.angular.x = 0;
   VelocityMessage.angular.y = 0;
   VelocityMessage.angular.z = 0;

   double distance_moved = 0.0;
   ros::Rate loop_rate(10); // we publish the velocity at 10 Hz (10 times a second)

   /*
   * First, we capture the initial transformation before starting the motion.
   * we call this transformation "init_transform"
   * It is important to "waitForTransform" otherwise, it might not be captured.
   */
   try{
       //wait for the transform to be found
       listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
       //Once the transform is found,get the initial_transform transformation.
       listener.lookupTransform("/base_footprint", "/odom", ros::Time(0), init_transform);
   }
   catch (tf::TransformException & ex){
       ROS_ERROR(" Problem %s", ex.what());
       ros::Duration(1.0).sleep();
   }

   do{
       /***************************************
       * STEP1. PUBLISH THE VELOCITY MESSAGE
       ***************************************/
       wheel_pub.publish(VelocityMessage);
       ros::spinOnce();
       loop_rate.sleep();
       /**************************************************
       * STEP2. ESTIMATE THE DISTANCE MOVED BY THE ROBOT
       *************************************************/
       try{
           //wait for the transform to be found
           listener.waitForTransform("/base_footprint", "/odom", ros::Time(0), ros::Duration(10.0) );
           //Once the transform is found,get the initial_transform transformation.
           listener.lookupTransform("/base_footprint", "/odom",ros::Time(0), current_transform);
       }
       catch (tf::TransformException & ex){
           ROS_ERROR(" Problem %s", ex.what());
           ros::Duration(1.0).sleep();
       }
       /*
       * Calculate the distance moved by the robot
       * There are two methods that give the same result
       */

       /*
       * Method 1: Calculate the distance between the two transformations
       * Hint:
       *    --> transform.getOrigin().x(): represents the x coordinate of the transformation
       *    --> transform.getOrigin().y(): represents the y coordinate of the transformation
       */
       //calculate the distance moved
       //cout<<"Initial Transform: "<<init_transform <<" , "<<"Current Transform: "<<current_transform<<endl;

       distance_moved = sqrt(pow((current_transform.getOrigin().x()-init_transform.getOrigin().x()), 2) +
               pow((current_transform.getOrigin().y()-init_transform.getOrigin().y()), 2));


   }while((distance_moved<distance)&&(ros::ok()));

   //finally, stop the robot when the distance is moved
   VelocityMessage.linear.x = 0;
   wheel_pub.publish(VelocityMessage);
}


double TurtleBot_Pattern::rotate(double angular_velocity, double radians, bool clockwise)
{

   //delcare a Twist message to send velocity commands
   geometry_msgs::Twist VelocityMessage;
   //declare tf transform listener: this transform listener will be used to listen and capture the transformation between
   // the odom frame (that represent the reference frame) and the base_footprint frame the represent moving frame
   tf::TransformListener TFListener;
   //declare tf transform
   //init_transform: is the transformation before starting the motion
   tf::StampedTransform init_transform;
   //current_transformation: is the transformation while the robot is moving
   tf::StampedTransform current_transform;
   //initial coordinates (for method 3)
   nav_msgs::Odometry initial_turtlebot_odom_pose;

   double angle_turned = 0.0;

   //validate angular velocity; ANGULAR_VELOCITY_MINIMUM_THRESHOLD is the minimum allowed
   angular_velocity = ((angular_velocity > ANGULAR_VELOCITY_MINIMUM_THRESHOLD) ? angular_velocity : ANGULAR_VELOCITY_MINIMUM_THRESHOLD);

   while(radians < 0) radians += 2 * M_PI;
   while(radians > (2 * M_PI)) radians -= 2 * M_PI;

   //wait for the listener to get the first message
   TFListener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));


   //record the starting transform from the odometry to the base frame
   TFListener.lookupTransform("base_footprint", "odom", ros::Time(0), init_transform);


   //the command will be to turn at 0.75 rad/s
   VelocityMessage.linear.x = VelocityMessage.linear.y = 0.0;
   VelocityMessage.angular.z = angular_velocity;
   if (clockwise) VelocityMessage.angular.z = -VelocityMessage.angular.z;

   //the axis we want to be rotating by
   tf::Vector3 desired_turn_axis(0, 0, 1);
   if (!clockwise) desired_turn_axis = -desired_turn_axis;

   ros::Rate rate(10.0);
   bool done = false;
   while (!done){
       //send the drive command
       wheel_pub.publish(VelocityMessage);
       rate.sleep();
       //get the current transform
       try
       {
           TFListener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));
           TFListener.lookupTransform("base_footprint", "odom", ros::Time(0), current_transform);
       }
       catch (tf::TransformException ex)
       {
           ROS_ERROR("%s", ex.what());
           break;
       }
       tf::Transform relative_transform = init_transform.inverse() * current_transform;
       tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
       angle_turned = relative_transform.getRotation().getAngle();

       if (fabs(angle_turned) < 1.0e-2) continue;
       if (actual_turn_axis.dot(desired_turn_axis ) < 0 )
           angle_turned = 2 * M_PI - angle_turned;

       if (!clockwise)
           VelocityMessage.angular.z = (angular_velocity-ANGULAR_VELOCITY_MINIMUM_THRESHOLD) * (fabs(radian2degree(radians-angle_turned) / radian2degree(radians))) + ANGULAR_VELOCITY_MINIMUM_THRESHOLD;
       else
           if (clockwise)
               VelocityMessage.angular.z = (-angular_velocity+ANGULAR_VELOCITY_MINIMUM_THRESHOLD) * (fabs(radian2degree(radians-angle_turned) / radian2degree(radians))) - ANGULAR_VELOCITY_MINIMUM_THRESHOLD;

       if (angle_turned > radians) {
           done = true;
           VelocityMessage.linear.x = VelocityMessage.linear.y = VelocityMessage.angular.z = 0;
           wheel_pub.publish(VelocityMessage);
       }
   }

   if (done) return angle_turned;
   return angle_turned;
}

/* makes conversion from radian to degree */
double TurtleBot_Pattern::radian2degree(double radianAngle){
   return (radianAngle * 57.2957795);
}

/* makes conversion from degree to radian */
double TurtleBot_Pattern::degree2radian(double degreeAngle){
   return (degreeAngle / 57.2957795);
}


int main(int argc, char **argv){

   //initialize the ROS node
   ros::init(argc, argv, "Turtlebot Pattern");

   double moving_speed = 0.3;
   double turning_speed = 0.1;
   double distance = 2.5;

   TurtleBot_Pattern turtlebot;

   turtlebot.movePattern(distance, moving_speed, turning_speed);
 
   return 0;
}
