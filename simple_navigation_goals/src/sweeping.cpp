#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <Grid.h>

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
  int sweep_idx;
  Grid field;
  //we'll send a goal to the robot to move 1 meter forward
  for (int i=0; i<field.map_height; i++) {
  	for (int j=0; j<field.map_width; j++) {	
	  goal.target_pose.header.frame_id = "map";
	  goal.target_pose.header.stamp = ros::Time::now();
	  if (i==0&&j==0) {
		  goal.target_pose.pose.position.x = field.getX(0);  
		  goal.target_pose.pose.position.y = field.getY(0);
		  goal.target_pose.pose.orientation.z = 0;
		  goal.target_pose.pose.orientation.w = 1;
		  ROS_INFO("Sending goal");
		  ac.sendGoal(goal);

		  ac.waitForResult();

		  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			  ROS_INFO("Hooray, the base has moved to the starting point");
		  else
			  ROS_INFO("The base failed to move for some reason");  
		  continue;
		  }  
	  if (i%2==0) sweep_idx=i*field.map_width+j;
	  else sweep_idx=(i+1)*field.map_width-j-1;
	  if (sweep_idx%field.map_width==field.map_width-1) {
	  	goal.target_pose.pose.orientation.z = sin(PI/4);
		goal.target_pose.pose.orientation.w = cos(PI/4);
		}
	  else if (sweep_idx%(2*field.map_width)>=0 && sweep_idx%(2*field.map_width)<field.map_width-1) {
	  	goal.target_pose.pose.orientation.z = 0;
		goal.target_pose.pose.orientation.w = 1;
		}
	  else {
	  	goal.target_pose.pose.orientation.z = 1;
		goal.target_pose.pose.orientation.w = 0;
		}
	  goal.target_pose.pose.position.x = field.getX(field.getC(sweep_idx));  
      goal.target_pose.pose.position.y = field.getY(field.getR(sweep_idx));
	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);
	  	
	  ac.waitForResult();

	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    ROS_INFO_STREAM("Hooray, the base moved forward "<<sweep_idx);
	  else
	    ROS_INFO("The base failed to move forward 1 meter for some reason");
	    }
	}
return 0;
}
