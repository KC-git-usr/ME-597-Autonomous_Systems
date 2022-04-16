execute these commands to auto map:

$export TURTLEBOT3_MODEL=waffle
$roslaunch final_project task_1.launch
$rosrun map_server map_saver -f /home/kc/catkin_ws/src/final_project/auto_scan_map/map

bugs to be fixed:
1. should i manually change lidar parameters in turtlebot3_waffle.gazebo.xarco and gmapping_params.yaml ?