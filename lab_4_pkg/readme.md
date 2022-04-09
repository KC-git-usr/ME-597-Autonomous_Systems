## Explanation of our work 
# astar_mapy.py 
- Creates all the needed classes for A* algorith. 
- Loads my_map file automatically as long as it's in the same directory as astar_mapy.py
- Use trigger function to send the global goal (from navigation.py) to astar_algorithm (in astar_map.py)
- Returns the calcualted path


# navigation.py 
-  We have a calibration step that re-orients the robot initially, using a PID controller, this runs once until the robot reaches the desired pose. 
- We update the astar algorithm with the desired goal position
- Move turtlebot using a low low level controller to the desired paths. 
- Conversion from gazebo distance to pixels is done through a_Star_path_planner function. We figured out the scaling of the map based on resolution. The only thing we didn't figure out the inital offset between map data and gazebo data. 