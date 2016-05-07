# IntroRoboticsFinalProject

### Team members

  * Andres Torres Garcia
  * Guoxiang Zhang
  * Jasmit Kakkar

### How to run
* 1 Run ros master
```
   roscore
```
* 2 Set the sim time to true
```
   rosparam set use_sim_time true
```
* 3 Run here ~/CSE180/src/robotics_course/urdfs
```
   rosrun gazebo_ros  gazebo factory_wo_p3at.sdf
```
* 4 Run here ~/CSE180/src/robotics_course/urdfs
```
   rosrun gazebo_ros spawn_model -file p3at_amr.urdf -urdf -z 1  -x 0 -y 0 -model pioneer3at
```
* 5 Run here ~/CSE180/src/robotics_course/launch_files
```
   roslaunch robotics_course run_full_simulation.launch
```
* 6 Run rviz if you want to see the particle filter with the map
```
   rviz
```
* 7 Run the map node, this will create adjacency.txt which includes the # of vertices their centers and an adjacency matrix
```
   rosrun finalproject map
```
* 8 Run the path node
```
   rosrun finalproject path
```
* 9 Run the navigation node and wait until the robot is localized.
```
   rosrun finalproject navigation
```
* 10 Run the testingNav node
```
   rosrun finalproject testingNav
```
### How to change goal location

Change GOAL_X and GOAL_Y in  finalproject/src/testNav.cpp (on the top of this file). 

The closes free point to the goal(0,0) is .75 and .75, but in the simulator it is out side the room, so we use (3,3) as our default goal as we did in lab6. 

## How did we slove those questions

### Andres Torres Garcia
I solved question 5 using a server and a client for the navigation. When the server navigation is up and running it won't receive any request for going to a particular waypoint until the robot get localized. The robot will turn in place to get localized. Once the robot is localized, the client will send the first waypoint of the path to follow to the goal. If it succed it will send the next waypoint and so on until the last waypoint, which is the goal, is sent. If for a particular reason there is an obstacle thorugh the path, the robot will try a number of times to reach the goal. If it fails it will move to the next goal and so on. If for a particular reason the robot gets lost during the navigation it will replan again.

### Guoxiang Zhang

#### Q3:Write a planner

I am using Dijkstra to find a path in the graph. 

First, my code read the adjacency matrix along with coordinates of the vertices from text file generated from Q2

Then std::priority_queue is used to find the path. Since in Dijkstra, the priority of a node would change, but std::priority_queue
do not provide a interface to change priority. I choose to keep a flag for each node: visited or not. When change of priority happen,
I only push a new element to the the priority queue. Then when dequeue, if the node is visited, It will just throw away that node. 
If the node is not visited, it will put this node to the close set.

#### Q4: Use the planner to determine a path

I implemented a service which can take a init pose and a goal pose as input, then return a nav_msgs/Path. 

Inside this service, it will find init and goal cell based on init and goal pose. Then use the corresponding vertices to get a path from Dijkstra. Then find corresponding centers of those vertices on the path. Then push those centers to a nav_msgs/Path then return it.











=========================================================================================================================
Execute this commands in order. If you have the simulator, the robot model and the particle filter set and running, you can skip to step 6.

1.- Run ros master

roscore

2.- Set the sim time to true

rosparam set use_sim_time true

3.- Run here ~/CSE180/src/robotics_course/urdfs

rosrun gazebo_ros  gazebo factory_wo_p3at.sdf

4.- Run here ~/CSE180/src/robotics_course/urdfs

rosrun gazebo_ros spawn_model -file p3at_amr.urdf -urdf -z 1  -x 0 -y 0 -model pioneer3at

5.- Run here ~/CSE180/src/robotics_course/launch_files

roslaunch robotics_course run_full_simulation.launch

6.- Run rviz if you want to see the particle filter with the map

rviz

7.- Run the map node, this will create adjacency.txt which includes the # of vertices their centers and an adjacency matrix

rosrun finalproject map

8.- Run the path node

rosrun finalproject path

9.- Run the navigation node and wait until the robot is localized.

rosrun finalproject navigation

//******Note: If you want to change the goal for the robot locate the defines that are located in the source file IntroRoboticsFinalProject/src/finalproject/src/testNav.cpp. The names of the defines are GOAL_X for the x coordinate and GOAL_Y for the y coordinate on the plane and compile it again: catkin_make. The closes free point to the goal(0,0) is .75 and .75 so, if you want to plan to that goal please use the point (.75, .75)

10.- Run the testingNav node

rosrun finalproject testingNav

//******Extra: If you want to use the same configuration that I'm using for the particle filter the file is located in IntroRoboticsFinalProject/src/finalproject/src/amcl.launch

//******Extra: To correctly kill Gazebo if you want to restart the simulation

killall -9 gzserver

Submit the whole folder including the package finalproject. In addition, provide a textual description explaining which questions you solved and how. Moreover, provide clear explanations on how to run your code. One submission per group is sufficient, as long as all team members are clearly identified in the submission.


If you want to change the goal for the robot locate the defines that are located in the source file IntroRoboticsFinalProject/src/finalproject/src/testNav.cpp. The names of the defines are GOAL_X for the x coordinate and GOAL_Y for the y coordinate on the plane and compile it again: catkin_make. The closes free point to the goal(0,0) is .75 and .75, but in the simulator it is out side the room, so we use (3,3) as our default goal as we did in previous lab. 

