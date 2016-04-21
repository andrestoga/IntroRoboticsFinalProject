/*
Jasmit Kakkar
Lab 5
*/

#include <ros/ros.h> 
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include <tf/transform_datatypes.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>



//ros::Publisher pub;
double roll, pitch, yaw;
double width, height;
int map[1760];
int twoDarr[400][440];

int Partition(int x, int y)
{
	int value = 0;

	for(int i = x; i<(x+10); i++){
		for(int e = y; e<(y+10); e++){
			if(twoDarr[i][e]==-1)
				value=100;
			if(twoDarr[i][e]==100)
				value=100;
		}
		
	}
	return value;

}

void scanMessageReceived(const nav_msgs::OccupancyGrid msg) {

	//msg.data
	width = msg.info.width;
	height = msg.info.height;
	//map = msg.data
	ROS_INFO_STREAM("width :" << width << "height :" << height);
	if(msg.data[0]==-1)
		ROS_INFO_STREAM("NULL");
	else{
	ROS_INFO_STREAM("notnull");	
	}

	
	int srcArrCounter = 0;
	int counter = 0;
	while(counter<399){
		for (int i =0; i<434; i++){
			
			twoDarr[counter][i]= msg.data[srcArrCounter];
			srcArrCounter++;	
	
		}
		for(int e = 434; e<440; e++){
			twoDarr[counter][e]= -1;
		}
		counter++;
	}
	for(int f = 0; f<440; f++){
		twoDarr[399][f] = -1;
	}
	//ROS_INFO_STREAM("srcarrcounter"<<srcArrCounter);
	int a, b, c, d;
	a = Partition(0,0);
	b = Partition(9,9);
	c = Partition(100,100);
	d = Partition(390,430);
	ROS_INFO_STREAM("partition" << a<<","<<b<<","<<c<<","<<d);
	


	int numPartition = 0;
	ROS_INFO_STREAM("partitions");
	
	for(int g = 0; g<=390; g+=10){
		for(int h = 0; h<=430; h+=10){
			//map[numPartition] = 0;
			map[numPartition] = Partition(g, h);
			
			ROS_INFO_STREAM(numPartition<<"  ,  "<<map[numPartition]);
			numPartition++;
		}
		
	}

	//ROS_INFO_STREAM("2darr:"<< twoDarr[0][0] <<  twoDarr[1][1]<<twoDarr[0][439]);
	
} 



int main(int argc,char **argv) {
	//this random generator will be used in the subscriber function
	srand(time(0));
	ROS_INFO_STREAM("running...");
	//here we initialize and create a node handle
	ros::init(argc,argv,"laser_subscriber");   
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(1000);
	
	ros::Subscriber sub = nh.subscribe("/map",1,&scanMessageReceived);
	while(ros::ok()){
	ros::spinOnce();

	}


	

}
