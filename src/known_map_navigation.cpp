//https://github.com/aniskoubaa/gaitech_edu/blob/master/src/turtlebot/navigation/map_navigation/map_navigation.cpp

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

/** function declarations **/
bool moveToGoal(double xGoal, double yGoal);
char choose();

/** declare the coordinates of interest **/
//rostopic echo /amcl_pose
double xStart = 0.00;
double yStart = 0.00;
double xDoor = 6.40;
double yDoor = 0.15;
double xCorner = 5.35;
double yCorner = 5.20;
double xHallway = 10.00;
double yHallway = -1.00;

bool goalReached = false;

int main(int argc, char** argv){
	ros::init(argc, argv, "known_map_navigation");
	ros::NodeHandle n;

	char choice = 'q';
	do{
		choice =choose();
		if (choice == '0'){
			goalReached = moveToGoal(xStart, yStart);
		}else if (choice == '1'){
			goalReached = moveToGoal(xDoor, yDoor);
		}else if (choice == '2'){
			goalReached = moveToGoal(xCorner, yCorner);
		}else if (choice == '3'){
			goalReached = moveToGoal(xHallway, yHallway);
		}
		if (choice!='q'){
			if (goalReached){
				ROS_INFO("Congratulations!");
				ros::spinOnce();
			}else{
				ROS_INFO("Hard Luck!");
			}
		}
	}while(choice !='q');
	return 0;
}

bool moveToGoal(double xGoal, double yGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/
	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}


char choose(){
	char choice='q';
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|PRESSE A KEY:"<<std::endl;
	std::cout<<"|'0': Starting Position "<<std::endl;
	std::cout<<"|'1': Door "<<std::endl;
	std::cout<<"|'2': Corner "<<std::endl;
	std::cout<<"|'3': Hallway "<<std::endl;
	std::cout<<"|'q': Quit "<<std::endl;
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|WHERE TO GO?";
	std::cin>>choice;

	return choice;
}