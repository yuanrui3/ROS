#include <ros/ros.h>
#include <algorithm>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>
#include <nav_msgs/OccupancyGrid.h>
#include <W1D.h>
#include <Grid.h>
#include <boost/random.hpp>
#include <boost/random/discrete_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <limits>
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::mt19937 base_generator;

//Target_distribution
static int8_t target_map[1800] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,0,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,0,0,0,100,100,100,0,0,0,0,0,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,0,0,0,0,0,0,100,100,100,100,0,0,0,0,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,0,0,0,0,0,0,0,0,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,0,0,0,0,0,0,0,0,0,0,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,0,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,0,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,0,0,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,0,0,0,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,0,0,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,0,100,100,100,100,100,100,0,100,100,100,0,0,0,0,0,0,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,0,100,100,100,100,100,100,100,100,100,100,0,0,0,0,0,0,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,0,0,100,100,100,100,100,100,100,100,100,100,0,0,0,0,0,0,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,100,0,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,0,0,0,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,0,0,0,0,0,0,0,0,0,100,100,100,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,100,0,0,0,0,0,0,0,0,0,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,100,100,100,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
;
double alpha[1800];
double beta[1800];
std::vector<double> aoi_coord;
std::vector<double> visited_aoi;
int exploration_map[1800]={};//unexplored:0; target_found:1; target_not_found:-1 
Grid field;
double X,Y;
int idx, row, col;

//Environment variables
static int reward = 10;
int get_reward = 0;
int no_reward = 0;
int num_grids = field.map_width*field.map_height;
int num_targets = 242;
double diagonal = sqrt(pow(field.map_width,2)+pow(field.map_height,2));

//Methods
bool edge_filtering= 1;
int method = 1;
	
double * imbinarize(double alpha[1800], double beta[1800]) {
	double alpha_min = *std::min_element(alpha,alpha+1800);
	double alpha_max = *std::max_element(alpha,alpha+1800);
	for (int i =0; i<1800; i++) alpha[i]=(alpha[i]-alpha_min)/(alpha_max-alpha_min);
	for (int i =0; i<1800; i++) {
		if (alpha[i]>0.05) beta[i]=1;
		}
  return beta;
	}
	
double ws_distance(std::vector<double> a, std::vector<double> b) {
	std::vector<double> aw(a.size(),1);
	std::vector<double> bw(b.size(),1);
	return wasserstein(a,aw,b,bw);
	}
	
int ThompsonSampling(int goal_idx) {
	int goal_col=field.getC(goal_idx);
	int goal_row=field.getR(goal_idx);
	int range = 1;
	base_generator gen;
	for (int i=0; i<field.map_height; i++) {
		for (int j=0; j<field.map_width; j++) {
			if (abs(i-goal_row)<=range&&abs(j-goal_col)) {
				if (target_map[goal_idx]==100) {
					alpha[i]+=reward;
					get_reward++;
					no_reward = 0;
					} else {
					beta[i]+=reward;
					get_reward = 0;
					no_reward++;
					}
				}
			}
		}
	double theta[1800];
	for (int i=0; i<1800; i++) {
		if (exploration_map[i]==-1) theta[i]=std::numeric_limits<double>::lowest();
		else if (get_reward>0) {
			boost::random::beta_distribution<> dist(alpha[i], beta[i]);	
  			double prior = dist(gen);
			double dist_penalty = sqrt(pow((field.getC(i)-goal_col),2)+pow((field.getR(i)-goal_row),2))/diagonal;
			theta[i]=prior+(1-dist_penalty)*get_reward;
			}
		else {
			boost::random::beta_distribution<> dist(alpha[i], beta[i]);	
  			double prior = dist(gen);
			double dist_penalty = sqrt(pow((field.getC(i)-goal_col),2)+pow((field.getR(i)-goal_row),2))/diagonal;
			theta[i]=prior+(1-dist_penalty)/no_reward;
			}
		}
	int max = *std::max_element(theta, theta+1800);
	return std::find(theta, theta+1800, max) - theta;
	}

/*boost::random::beta_distribution<> dist(alpha, beta);	
  base_generator gen;
  prior = dist(gen);
*/

int goal_idx = 0;
geometry_msgs::Point position;
geometry_msgs::Quaternion orientation;

//Subscribe to initial pose
bool occupied;
bool newmsg=false;
void poseReceived(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  position = msg.pose.pose.position;
  orientation = msg.pose.pose.orientation;
  ROS_INFO("reading pose......");
  ROS_INFO_STREAM("position: "<<position<<"orientation: "<<orientation);
  newmsg=true;
}

int main(int argc, char** argv){
  for (int i=0; i<1800; i++) {
	if (target_map[i]==100) aoi_coord.push_back(i);
  alpha[i] = 1;
  beta[i] = 1;
	}
  
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/amcl_pose",1000,&poseReceived);
  while (!newmsg) ros::spinOnce();
  //newmsg=false;
  col = field.getRow(position.x);
  row = field.getCol(position.y);
  idx = field.getIndex(row,col);
  if (target_map[idx] == 0) {
  	occupied = false;
  	ROS_INFO("No target");
  	} else {
  	occupied = true;
  	ROS_INFO("Target found");
  	}
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to the starting point
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = field.getX(0);
  goal.target_pose.pose.position.y = field.getY(0);
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray");
  else
    ROS_INFO("The base failed");

  visited_aoi.push_back(goal_idx);
  goal_idx = ThompsonSampling(goal_idx);

  while (visited_aoi.size() <= num_targets) {
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = field.getX(field.getC(goal_idx));  
  goal.target_pose.pose.position.y = field.getY(field.getR(goal_idx));
  goal.target_pose.pose.orientation.z = 1.0;
  goal.target_pose.pose.orientation.w = 0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray");
  else
    ROS_INFO("The base failed");

  visited_aoi.push_back(goal_idx);
  goal_idx = ThompsonSampling(goal_idx);
  }
    
    /* (param-velocity-adjustment-approach deprecated
  double maxvel, minvel;
  n.getParam("/move_base/TrajectoryPlannerROS/max_vel_x", maxvel);
  n.getParam("min_vel_x", minvel);
  ROS_INFO("%f%f",maxvel,minvel);*/
  double ws_d = ws_distance(aoi_coord, visited_aoi);
  ROS_INFO_STREAM("distance: "<<ws_d);
  ros::spinOnce();
  return 0;
}


