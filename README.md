# IntroRoboticsFinalProject
Important******: Before beginning rename the files navigation (copy).cpp to navigation.cpp and testingNav (copy).cpp to testingNav.cpp

Execute this commands in order

//Run ros master
roscore

//Set the sim time to true
rosparam set use_sim_time true

//Run here ~/CSE180/src/robotics_course/urdfs
rosrun gazebo_ros  gazebo factory_wo_p3at.sdf

//Run here ~/CSE180/src/robotics_course/urdfs
rosrun gazebo_ros spawn_model -file p3at_amr.urdf -urdf -z 1  -x 0 -y 0 -model pioneer3at

//Run here ~/CSE180/src/robotics_course/launch_files
roslaunch run_full_simulation.launch

//Run rviz if you want to see the particle filter with the map
rviz

//Run the path node
rosrun finalproject path

//Run the navigation node and wait until the robot is localized
rosrun finalproject navigation

//Run the testingNav node
rosrun finalproject testingNav

//******Extra: To correctly kill Gazebo if you want to restart the simulation
killall -9 gzserver
