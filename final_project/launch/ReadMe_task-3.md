**Instructions**

1. Open final_project/src/astar_map.py, go to line 352, make sure the directory is correct
2. Do the same for line 353
3. Execute this command:
    $roslaunch final_project task_3.launch
4. To increase speed of simulation:
    *Switch to Gazebo window
    *click physics->propoerty->real time
    *set it to 5000

To fix:
1. clean up print statements
2. multiple white objects, pick the closest -> get centroid -> get lidar data from that direction

1. Idea 1: perspective transformation from image and lidar
2. Idea 2: get lidar point cloud data, if circular...

To Do:
1. change map name
2. inc speed of bot, tune PID- rqt plot
3. comple readme's
4. add additional cameras?
5. resize image in detect_dyn_obs?