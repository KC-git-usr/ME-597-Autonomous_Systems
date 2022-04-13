execute these commands to auto map:

$export TURTLEBOT3_MODEL=waffle
$roslaunch final_project ME597_final_project.launch
$roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
$rosrun final_project task_1.py
$rosrun map_server map_saver -f /home/kc/catkin_ws/src/final_project/maps/map

bugs to be fixed:
1. gmapping not working correctly when i automate launch in task_1.launch
2. should i manually change lidar parameters in turtlebot3_waffle.gazebo.xarco and gmapping_params.yaml ?