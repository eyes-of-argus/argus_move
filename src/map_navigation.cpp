#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"

#include <iostream> 
using namespace std; 

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
#define MAX_DISTANCE 2.0      //2.0 meters (4 blocks)
#define REDUCE_DISTANCE 0.5   //0.5 meter (1 block)

class PathPlanning{
public:
  PathPlanning() : ac("move_base", true){
    distance = MAX_DISTANCE;
    stepper = REDUCE_DISTANCE;
    reverse_distance = 0;
    temp_distance = 0;

    array_size = ((2 / REDUCE_DISTANCE) * MAX_DISTANCE) + 1;
    goal_index = 0;
    sit = false;
    cancel_trigger = 0;

    simulator_sub = node.subscribe("/simulator_test", 1, &PathPlanning::scanHandler,this);

    //Clear up the server
    ac.cancelAllGoals();

    //Calculate for an array of pattern coordinates
    calculateCoordinates();

    for (int i = 0; i < array_size; i++){
       ROS_INFO_STREAM("X: " << goal[i].target_pose.pose.position.x <<"\t"<< "Y: "<< goal[i].target_pose.pose.position.y <<"\n");        
    }

    while (!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    } 
  }

  void calculateCoordinates(){
      goal = new move_base_msgs::MoveBaseGoal[array_size];

      for (int i = 0; i < array_size; i++){
          setupCoordinates(i);
          
          if(i % 4 == 2)
            distance -= stepper;
          
          if(i % 4 == 1){
            temp_distance = reverse_distance;
            reverse_distance += stepper;
          }
      }
  }

  void setupCoordinates(int i){
    int direction = i % 4;
    goal[i].target_pose.header.frame_id = "map";

    double theta = 90.0;
    if (direction == 1)
      theta = 180.0;

    if (direction == 2)
      theta = 270.0;

    if (direction == 3)
      theta = 360.0;

    double radians = theta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);
    geometry_msgs::Quaternion qMsg;tf::quaternionTFToMsg(quaternion, qMsg);
    goal[i].target_pose.pose.orientation= qMsg;

    switch (direction){
      case 0:
        goal[i].target_pose.pose.position.x = distance;
        goal[i].target_pose.pose.position.y = reverse_distance;
        break;
      case 1:
        goal[i].target_pose.pose.position.x = distance;
        goal[i].target_pose.pose.position.y = distance;
        break;
      case 2:
        goal[i].target_pose.pose.position.x = temp_distance;
        goal[i].target_pose.pose.position.y = distance;
        break;
      case 3:
        goal[i].target_pose.pose.position.x = temp_distance;
        goal[i].target_pose.pose.position.y = reverse_distance;
        break;
      }
  }

  void sendCoordinates(int index = 0){
    if (index < array_size){
      ac.sendGoal(goal[index], boost::bind(&PathPlanning::goalCallBack, this, _1), MoveBaseClient::SimpleActiveCallback());
    }
  }

  void scanHandler(const std_msgs::Bool::ConstPtr& stop){
    //  if(stop->data == true){
    //     sit = true;
    //     cancel_trigger++;
    //  }
    //  else{
    //     sit = false;
    //     cancel_trigger = 0;
    //  }

    //  //ONLY send cancel goal once when we keep continuously receice pause command 
    //  if (cancel_trigger == 1){
    //     ac.cancelGoal();
    //  }
     
  }

  void goalCallBack(const actionlib::SimpleClientGoalState &state){
    ROS_INFO("CALL BACK: Finished in state [%s]", state.toString().c_str());

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("FINISHED");
      goal_index++;

      ros::Rate rate(10);
      ros::Duration turn_timeout(2);
      ros::Time start_time = ros::Time::now();
      while (ros::Time::now() - start_time < turn_timeout){
          rate.sleep();
      }

      sendCoordinates(goal_index);

    }
      
    if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED || ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
      ROS_INFO("CANCELLED");

      while(sit){
      } 
      ROS_INFO("RESEND");
      sendCoordinates(goal_index);
    }

  }

private:
  move_base_msgs::MoveBaseGoal * goal;
  MoveBaseClient ac;

  double distance;
  double reverse_distance;
  double temp_distance;
  double stepper;
  int array_size;
  int goal_index;
  bool sit;
  int cancel_trigger;

  ros::NodeHandle node;
  ros::Subscriber simulator_sub;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "Turtlebot Path Planning");
  PathPlanning turtlebot;

  turtlebot.sendCoordinates();

  ros::spin();
  return 0;
}