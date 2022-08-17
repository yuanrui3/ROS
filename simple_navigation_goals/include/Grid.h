#include <math.h>
#include <geometry_msgs/Pose.h>
#define PI 3.1415926
class Grid {
   public:
    double grid_full = 0.2;
    double grid_half = 0.1;
    double orig_X = 8.733;
    double orig_Y = 9.123;
    //double orig_X = 6.16;
    //double orig_Y = 6.45;
    int map_height=36;
    int map_width=50;
	int getIndex(int row, int col) {
	return row*map_width+col;
	}
	double getX(int n) {
	return (n+0.5)*grid_full+orig_X;
	}
	double getY(int n) {
	return (n+0.5)*grid_full+orig_Y;
	}
    int getRow(double Y) {
      return (Y-orig_Y)/grid_full;
    }
    int getCol(double X) {
      return (X-orig_X)/grid_full;
    }
    int getR(int index) {
      return index/map_width;
      }
    int getC(int index){
      return index%map_width;
      }
    geometry_msgs::Pose goalPose(int goal_col, int goal_row, double cur_X, double cur_Y) {
    	geometry_msgs::Pose goal_pose;
    	double goal_X = getX(goal_col);
    	double goal_Y = getY(goal_row);
    	if (cur_X<goal_X&&cur_Y<goal_Y) {
    		goal_pose.position.x=goal_X-grid_half;
    		goal_pose.position.y=goal_Y-grid_half;
    		goal_pose.orientation.z=sin(PI/8);
    		goal_pose.orientation.w=cos(PI/8);
    		}
	else if (cur_X>=goal_X&&cur_Y<=goal_Y) {
    		goal_pose.position.x=goal_X+grid_half;
    		goal_pose.position.y=goal_Y-grid_half;
    		goal_pose.orientation.z=sin(3*PI/8);
    		goal_pose.orientation.w=cos(3*PI/8);
    		}
    	else if (cur_X>=goal_X&&cur_Y>goal_Y) {
    		goal_pose.position.x=goal_X+grid_half;
    		goal_pose.position.y=goal_Y+grid_half;
    		goal_pose.orientation.z=sin(5*PI/8);
    		goal_pose.orientation.w=cos(5*PI/8);
    		}
    	else {
    		goal_pose.position.x=goal_X-grid_half;
    		goal_pose.position.y=goal_Y+grid_half;
    		goal_pose.orientation.z=sin(7*PI/8);
    		goal_pose.orientation.w=cos(7*PI/8);
    		}  
    	return goal_pose;		
   	}
	};
