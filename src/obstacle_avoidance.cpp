#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/BumperEvent.h"


#define _USE_MATH_DEFINES
#include <cmath>

#include <sstream>

using namespace std;


class ObstacleAvoidance{
  public:
    ObstacleAvoidance();
    void demoDriver();

  private:
    void runStraight();
    void reverse();
    void dodgyTurn(double angular);
    void scanHandler(const sensor_msgs::LaserScan::ConstPtr& scan);
    void setVelocity(double linear, double angular);
    void turnDecision(float left_range, float right_range);
    void pause();

    ros::NodeHandle node;
    ros::Publisher wheel_pub;
    ros::Subscriber scan_sub;

    geometry_msgs::Twist vel_msg;
    ros::Time start_time;

    bool keep_moving;      //This is later for when we want it to stop
    double turn_right;
    double turn_left;
    double linear;
};


// Constructor: set up the publisher(s) and subsriber(s)
ObstacleAvoidance::ObstacleAvoidance(){
    wheel_pub = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 2);
    scan_sub = node.subscribe("/scan", 1, &ObstacleAvoidance::scanHandler,this);

    turn_right = -0.4;
    turn_left = 0.4;
    linear = 0.1;
}

void ObstacleAvoidance::demoDriver(){
    ROS_INFO("Start moving");
    ros::Rate rate(10);

    while (ros::ok()) {
        runStraight();
        // Need to call this function often to allow ROS to process incoming messages
        ros::spinOnce(); 
        rate.sleep();
    }
}


void ObstacleAvoidance::runStraight(){
    setVelocity(linear, 0.00);
    wheel_pub.publish(vel_msg);
}


void ObstacleAvoidance::dodgyTurn(double angular){
    ros::Rate rate(10);
    setVelocity(0.0, angular);

    ros::Duration turn_timeout(3);
    start_time = ros::Time::now();
    
    while (ros::Time::now() - start_time < turn_timeout){
        wheel_pub.publish(vel_msg);
        rate.sleep();
    }
}


void ObstacleAvoidance::scanHandler(const sensor_msgs::LaserScan::ConstPtr& scan){
    float angle_min = scan->angle_min;
    float angle_max = scan->angle_max;
    float angle_increment = scan->angle_increment;

    int max_index = floor(((angle_max - angle_min)/angle_increment)+1);

    //ranges[0] is almost always nan
    float right_range = scan->ranges[20];
    float mid_range = scan->ranges[floor(max_index/2)];
    float left_range = scan->ranges[max_index-20];

   
    ROS_INFO_STREAM("midrange: " << mid_range);

    //Too close for Kinect or out of range
    if(mid_range < 0.5 || isnan(mid_range)){
        ROS_INFO_STREAM("right_range: " << right_range <<"\t"<< "left_range: "<< left_range);
        //float min_range = std::min(std::min(right_range, left_range), mid_range);
        
        //if can't read all 3 = probably too close and got into a U corner
        if ((isnan(left_range) || left_range < 0.5) && (isnan(right_range) || right_range < 0.5)){
          dodgyTurn(turn_right);
          ROS_INFO("ASSUMED U SHAPED CORNER!!!! TURN RIGHT TO GET OUT !!!");
        }
        //if only the left and mid isnan = right seeing stuffs => just turn right
        else if ((isnan(left_range) || left_range < 0.5)){
          dodgyTurn(turn_right);
          ROS_INFO("Probably can turn Right !!!! TURN RIGHT");
        }
        //if only the right and mid isnan = left seeing stuffs => just turn left
        else if ((isnan(right_range) || right_range < 0.5)){
          dodgyTurn(turn_left);
          ROS_INFO("Probably can turn Left!!!! TURN LEFT");
        }
        // if you can see both the right and left, just turn to the side that has more room
        else{
          turnDecision(left_range, right_range);
        }
    }
}

void ObstacleAvoidance::setVelocity(double linear, double angular){
    vel_msg.linear.x = linear;
    vel_msg.angular.z = angular;
}

void ObstacleAvoidance::turnDecision(float left_range, float right_range){
    if (right_range < left_range){
      //We turning left here
      dodgyTurn(turn_left);
      ROS_INFO("TURN LEFT");
    }
    else{
      //We turning right then
      dodgyTurn(turn_right);
      ROS_INFO("TURN RIGHT");
    }
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "ObstacleAvoidance");

    ObstacleAvoidance demo;
    
    demo.demoDriver();

    return 0;
};

