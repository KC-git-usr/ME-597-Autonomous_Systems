**Instructions**

1. Open final_project/src/astar_map.py, go to line 352, make sure the directory is correct
2. Do the same for line 353
3. Execute this command:
    $roslaunch final_project task_3.launch
    $rosrun final_project dynamic_obstacles.py
    $rosrun final_project detect_dyn_obs.py
    $rosrun final_project task_3.py
4. To increase speed of simulation:
    *Switch to Gazebo window
    *click physics->propoerty->real time
    *set it to 5000

To fix:
1. clean up print statements
2. multiple white objects, pick the closest -> get centroid -> get lidar data from that direction

1. Idea 1: perspective transformation from image and lidar
2. Idea 2: get lidar point cloud data, if circular...