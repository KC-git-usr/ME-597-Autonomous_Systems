execute these commands to auto map:

$export TURTLEBOT3_MODEL=waffle
$roslaunch final_project task_1.launch
$rosrun map_server map_saver -f /home/kc/catkin_ws/src/final_project/maps/map

bugs to be fixed:
1. make the speed proportional
2. last minute change of lidar dist to 13m (was not aware), ttbot needs to be tuned for this range