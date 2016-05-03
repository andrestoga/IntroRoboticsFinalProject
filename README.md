# IntroRoboticsFinalProject
Important******: Before beginning rename the files navigation (copy).cpp to navigation.cpp and testingNav (copy).cpp to testingNav.cpp

Execute this commands in order

1.- Run ros master

roscore

2.- Set the sim time to true

rosparam set use_sim_time true

3.- Run here ~/CSE180/src/robotics_course/urdfs

rosrun gazebo_ros  gazebo factory_wo_p3at.sdf

4.- Run here ~/CSE180/src/robotics_course/urdfs

rosrun gazebo_ros spawn_model -file p3at_amr.urdf -urdf -z 1  -x 0 -y 0 -model pioneer3at

5.- Run here ~/CSE180/src/robotics_course/launch_files

roslaunch run_full_simulation.launch

6.- Run rviz if you want to see the particle filter with the map

rviz

7.- Run the path node

rosrun finalproject path

8.- Run the navigation node and wait until the robot is localized

rosrun finalproject navigation

9.- Run the testingNav node

rosrun finalproject testingNav

//******Extra: To correctly kill Gazebo if you want to restart the simulation

killall -9 gzserver
