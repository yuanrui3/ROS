#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <Grid.>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal goal;
  double prev_X;
  double prev_Y;
  int sweep_idx;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  for (int i=0; i<Grid.map_height; i++) {
  	for (int j=0; j<Grid.map_width; j++) {	
	  goal.target_pose.header.stamp = ros::Time::now();
	  if (i==0&&j==0) {
		  goal.target_pose.pose.position.x = Grid.getX(0);  
		  goal.target_pose.pose.position.y = Grid.getY(0);
		  goal.target_pose.pose.orientation.z = 0;
		  goal.target_pose.pose.orientation.w = 1;
		  prev_X=goal.target_pose.pose.position.x;
		  prev_Y=goal.target_pose.pose.position.y;
		  continue;
		  }
	  if (i%2==0) sweep_idx=i*Grid.map_width+j;
	  else sweep_id=(i+1)*Grid.map_width-j;
	  goal = goalPose(Grid.getC(sweep_idx),Grid.getR(sweep_idx),prev_X,prev_Y);

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  ac.waitForResult();

	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    ROS_INFO("Hooray, the base moved 1 meter forward");
	  else
	    ROS_INFO("The base failed to move forward 1 meter for some reason");
	    }
	}

  return 0;
}
