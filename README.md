# ROS
The directory *simple_navigation_goals* contains the ROS package. There is no need to download any other files.

Inside *src* directory, I have provided four ROS nodes. <occupancy_grid_map> publishes ROS message occupancy_grid_map. It's not useful for now. 
<test_goal> tests goal sending command.
<simple_navigation_goals> runs the Thompson Sampling algorithm.
<sweeping> runs the sweeping algorithm.

In a terminal(preferrable on the laptop), run __roslaunch dingo_navigation amcl_demo.launch map_file:=/path/to/my/map.yaml__
This launches the <amcl_demo> node.
Then open a new terminal and run __roslaunch dingo_viz view_robot.launch config:=localization__
This enables Rviz visulization.
Then open a new terminal and run __roslaunch simple_navigation_goals replacethiswithoneofthefournodes__
This starts one of the four nodes.

For the abovementioned, you can also refer to Dingo documentation: https://www.clearpathrobotics.com/assets/guides/melodic/dingo/navigation.html
Follow the instruciton under "Navigation with a Map".

Grid parameters are set up in the header file <Grid.h> inside *include* directory.
