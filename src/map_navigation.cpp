<<<<<<< Updated upstream
/*
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  std::vector<move_base_msgs::MoveBaseGoal> goal;
  goal[0].target_pose.pose.position.x = 2;
  goal[0].target_pose.pose.position.y = 0;
  goal[0].target_pose.pose.orientation.w = 1;

  goal[1].target_pose.pose.position.x = 2;
  goal[1].target_pose.pose.position.y = 2;
  goal[1].target_pose.pose.orientation.w = 1;

  goal[2].target_pose.pose.position.x = 0;
  goal[2].target_pose.pose.position.y = 2;
  goal[2].target_pose.pose.orientation.w = 1;

  goal[3].target_pose.pose.position.x = 0;
  goal[3].target_pose.pose.position.y = 0;
  goal[3].target_pose.pose.orientation.w = 1;

  for (int i = 0; i < goal.size(); i++){
        ac.sendGoal(goal[i]); // Current Goal
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Reached Goal successfully!");
        else
            ROS_INFO("Failed!");
  }
  return 0;
}
*/    



  //https://edu.gaitech.hk/turtlebot/send-goals-nav-stack.html
  #include <ros/ros.h>
  #include <move_base_msgs/MoveBaseAction.h>
  #include <actionlib/client/simple_action_client.h>

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  int main(int argc, char** argv){
    ros::init(argc, argv, "map_navigation");
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up and then start the process
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    //This is where you create the goal to send to move_base using move_base_msgs::MoveBaseGoal messages to tell the robot to move one meter forward in the coordinate frame.
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.2;
    goal.target_pose.pose.orientation.w = 0.2;

    ROS_INFO("Sending goal");

    //this command sends the goal to the move_base node to be processed
    ac.sendGoal(goal);

    //After finalizing everything you have to wait for the goal to finish processing
    ac.waitForResult();

    //here we check for the goal if it succeded or failed and send a message according to the goal status.
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
    else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

    return 0;
  }
=======
// // FOR NORMAL TESTING
// //http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
// #include <ros/ros.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// int main(int argc, char** argv){
//   ros::init(argc, argv, "map_navigation");

//   //tell the action client that we want to spin a thread by default
//   MoveBaseClient ac("move_base", true);

//   //wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }

//   move_base_msgs::MoveBaseGoal goal;

//   //we'll send a goal to the robot to move 1 meter forward
//   // goal.target_pose.header.frame_id = "base_footprint";
//   // goal.target_pose.header.stamp = ros::Time::now();
//   // goal.target_pose.pose.position.x = 4.0;
//   // goal.target_pose.pose.position.y = 0.0;
//   // goal.target_pose.pose.orientation.w = 1.0;

//   ROS_INFO("Sending goal");
//   ac.sendGoal(goal);

//   //ac.cancelGoal();
//   ac.waitForResult();

//   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     ROS_INFO("Hooray, the base moved 4 meter forward");
//   else
//     ROS_INFO("The base failed to move forward 4 meter for some reason");

//   return 0;
// }

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Bool.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PathPlanning{
public:
  PathPlanning() : ac("move_base", true){
    distance = 2.0;
    reverse_distance = 0;
    stepper = 0.5;

    goal_index = 0;
    sit = false;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.orientation.w = 1.0;

    simulator_sub = node.subscribe("/simulator_test", 1, &PathPlanning::scanHandler,this);

    while (!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
  }

  void scanHandler(const std_msgs::Bool::ConstPtr& stop){
    ROS_INFO_STREAM("DATA: " << stop->data);
     if(stop->data){
        sit = true;
        ROS_INFO("SIT TRUE");
        ac.cancelGoal();
     }
     else{
        sit = false;
        ROS_INFO("SIT FALSE");
     }
     
  }

  void move(int direction){
    switch (direction){
    case 0:
      goal.target_pose.pose.position.x = distance;
      goal.target_pose.pose.position.y = reverse_distance;
      break;
    case 1:
      goal.target_pose.pose.position.x = distance;
      goal.target_pose.pose.position.y = distance;
      break;
    case 2:
      goal.target_pose.pose.position.x = reverse_distance;
      goal.target_pose.pose.position.y = distance;
      break;
    case 3:
      goal.target_pose.pose.position.x = reverse_distance;
      goal.target_pose.pose.position.y = reverse_distance;
      break;
    }

    ac.sendGoal(goal, boost::bind(&PathPlanning::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
    ac.waitForResult();
  }


  void movePattern(){
    int trigger = 2;

    while (distance > 0){
      for (int i = 0; i < 4 && distance > 0; i++) {
        goal_index = i;
        move(i);

        if (i == trigger){
          distance -= stepper;

          if (trigger == 2)
            trigger = 0;
          else
            trigger = 2;
        }
        ROS_INFO_STREAM("\nLOOP COUNT: " << i << "\n");
        ros::spinOnce();
      }
      reverse_distance += stepper;
    }
  }

  //  void doStuff(){
  //      goal.target_pose.header.frame_id = "map";
  //      goal.target_pose.header.stamp = ros::Time::now();
  //      goal.target_pose.pose.position.x = 2.0;
  //      goal.target_pose.pose.position.y = 0.0;
  //      goal.target_pose.pose.orientation.w = 1.0;

  //      ac.sendGoal(goal, boost::bind(&MyNode::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());

  //      ros::Rate loop_rate(10);
  //      ros::Time starting_time = ros::Time::now();
  //      ros::Duration pause_time = ros::Duration(3);

  //      while (ros::Time::now() - starting_time < pause_time)
  //      {
  //          ROS_INFO("Inside FIRST stop LOOP");
  //          loop_rate.sleep();
  //      }

  //      ac.cancelGoal();
  //      //ac.cancelAllGoals();
  //      ROS_INFO("CANCEL ALL GOALS");

  //      starting_time = ros::Time::now();
  //      while (ros::Time::now() - starting_time < pause_time)
  //      {
  //          ROS_INFO("Inside SECOND stop LOOP");
  //          loop_rate.sleep();
  //      }

  //      goal.target_pose.header.frame_id = "map";
  //      goal.target_pose.header.stamp = ros::Time::now();
  //      goal.target_pose.pose.position.x = 2.0;
  //      goal.target_pose.pose.position.y = 0.0;
  //      goal.target_pose.pose.orientation.w = 1.0;

  //      ac.sendGoal(goal, boost::bind(&MyNode::doneCb, this, _1), MoveBaseClient::SimpleActiveCallback());
  //  }

  void doneCb(const actionlib::SimpleClientGoalState &state){
    ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("FINISHED");

    if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
      ROS_INFO("CANCELED");
      ros::Rate loop_rate(10);
      while(sit){
        loop_rate.sleep();
      }
      move(goal_index);
      ROS_INFO("RESEND");
    }
  }

private:
  move_base_msgs::MoveBaseGoal goal;
  MoveBaseClient ac;
  double distance;
  double reverse_distance;
  double stepper;
  int goal_index;
  bool sit; 

  ros::NodeHandle node;
  ros::Subscriber simulator_sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Turtlebot Path Planning");
  PathPlanning turtlebot;
  // my_node.doStuff();

  turtlebot.movePattern();

  ros::spin();
  return 0;
}

/* map = absolute coordinates : Testing = right side facing the window
  NOTE: spin circle too much since it needs the map*/
// move_base_msgs::MoveBaseGoal goal[4]

// goal[0].target_pose.header.frame_id = "map";
// goal[0].target_pose.pose.position.x = 2.0;
// goal[0].target_pose.pose.position.y = 0;
// goal[0].target_pose.pose.orientation.w = 1;

// goal[1].target_pose.header.frame_id = "map";
// goal[1].target_pose.pose.position.x = 2.0;
// goal[1].target_pose.pose.position.y = 2.0;
// goal[1].target_pose.pose.orientation.w = 1;

// goal[2].target_pose.header.frame_id = "map";
// goal[2].target_pose.pose.position.x = 0;
// goal[2].target_pose.pose.position.y = 2.0;
// goal[2].target_pose.pose.orientation.w = 1;

// goal[3].target_pose.header.frame_id = "map";
// goal[3].target_pose.pose.position.x = 0;
// goal[3].target_pose.pose.position.y = 0;
// goal[3].target_pose.pose.orientation.w = 1;
>>>>>>> Stashed changes
