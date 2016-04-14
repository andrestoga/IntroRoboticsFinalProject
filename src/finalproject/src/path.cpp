#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
# include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>

int main(int argc,char **argv) {
	ros::init(argc,argv,"path");
	ros::NodeHandle nh;

	// write a text file

	/*
	std::ofstream myfile("example.txt", std::ios::out | std::ios::binary);
	myfile << "Writing this to a file.\n";
	myfile.close();*/

	// read a text file
	std::ifstream infile("graph_example.txt");
  	if(!infile.is_open())
	{
    	ROS_INFO("file not found!");
    	ros::shutdown();
  	}
  	else
  	{
  		ROS_INFO("File found!");
  	}




	    // pub = nh.advertise<geometry_msgs::Twist>(
        //"/cmd_vel", 1000);
    //std::srand(std::time(0));
    //ros::Subscriber sub_est = nh.subscribe("/amcl_pose", 1, & estMsgReced);
	//ros::Subscriber sub = nh.subscribe("/scan", 1, & scanMsgReced);
	ros::spin();


}
