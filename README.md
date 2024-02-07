roscore 
cd ~/catkin_ws
catkin_make
roslaunch mie443_contest1 turtlebot_world.launch world:=1
roslaunch mie443_contest1 gmapping.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
rosrun mie443_contest1 contest1


roslaunch turtlebot_teleop keyboard_teleop.launch
rosrun mie443_contest1 contest1
