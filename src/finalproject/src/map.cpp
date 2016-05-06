/*
Jasmit Kakkar
Final Project
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
#include <iostream>
#include <fstream>



ros::Subscriber sub;
int callback = 1;
double roll, pitch, yaw;
double width, height;
int map[1760];
int partTwoDarr[40][44];
int twoDarr[400][440];
int adjacency[1384][1384];

std::ofstream myfile;

//the structure for the map data holding open neighbors and center values.
struct adj
{
	int number;
	int value;
	int left;
	int right;
	int up;
	int down;
	double centerX;
	double centerY;
};

adj ptr[40][44];

//partition will take an index and partition a 10x10 aquare from that point and determine its state.
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

	//revelant data
	width = msg.info.width;
	height = msg.info.height;
	//map = msg.data
	//ROS_INFO_STREAM("res:"<< msg.info.resolution);
	//ROS_INFO_STREAM("origin"<< msg.info.origin.position.x <<","<<msg.info.origin.position.y);
	//ROS_INFO_STREAM("width :" << width << "height :" << height);


	//recieve data from map and store
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
	

	
	int freeCells = 0;

	int numPartition = 0;
	//ROS_INFO_STREAM("partitions");
	double tempX = 0.25;
	double tempY = 0.25;
	
	//partition the 400x440 array to 40x44
	for(int g = 0; g<=390; g+=10){
		for(int h = 0; h<=430; h+=10){
			partTwoDarr[(g/10)][(h/10)] = Partition(g, h);
			ptr[(g/10)][(h/10)].value = Partition(g, h);

			ptr[(g/10)][(h/10)].centerX = tempX;
			ptr[(g/10)][(h/10)].centerY = tempY;

			map[numPartition] = Partition(g, h);
			if(map[numPartition]==0)
				freeCells++;
			tempX += 0.5;
			//ROS_INFO_STREAM(numPartition<<"  ,  "<<map[numPartition]);
			numPartition++;
		}
	tempX = 0.25;
	tempY += 0.5;
		
	}
	//ROS_INFO_STREAM("free cells:"<<freeCells);

	//initialize some variables and number off cells based on open state #1384
	int number = 1;
	for(int c = 0; c<40; c++){
		for(int v = 0; v<44; v++){
			ptr[c][v].left = -1;
			ptr[c][v].right = -1;
			ptr[c][v].up = -1;
			ptr[c][v].down = -1;
			if(ptr[c][v].value == 0){
				ptr[c][v].number = number;
				number++;
			}
			else{
				ptr[c][v].number = -1;
			}
		}
	}

	for(int d = 0; d<1384; d++){
		for(int f = 0; f<1384; f++){
			adjacency[d][f] = 0;
		}
	}
	
	
	
	//these loops check if an open vertex has neighbors it can traverse to.
	for(int q = 0; q<40; q++){
		for(int w = 0; w<44; w++){
			if(ptr[q][w].value == 0)
			{
				if(ptr[q-1][w].value == 0){
					if((q-1)>=0&&(q-1)<=40&&(w)>=0&&(w)<=44){
						ptr[q][w].up = (ptr[q-1][w].number);
					}
				
				}
				if(ptr[q+1][w].value == 0){
					if((q+1)>=0&&(q+1)<=40&&(w)>=0&&(w)<=44){
						ptr[q][w].down = (ptr[q+1][w].number);
					}
				}
				if(ptr[q][w-1].value == 0){
					if((q)>=0&&(q)<=40&&(w-1)>=0&&(w-1)<=44){
						ptr[q][w].left = (ptr[q][w-1].number);
					}
				}
				if(ptr[q][w+1].value == 0){
					if((q)>=0&&(q)<=40&&(w+1)>=0&&(w+1)<=44){
						ptr[q][w].right = (ptr[q][w+1].number);
					}
				}
				
				
			}
		}
	}	

	//these loops will fill in the adjacency matrix
	int ctr = 0;	
	for(int z = 0; z<40; z++){
		for(int x = 0; x<44; x++){
			if(ptr[z][x].value ==0){
				if(ptr[z][x].left != -1){
					adjacency[ctr][(ptr[z][x].left)-1] = 1;
				}
				if(ptr[z][x].right != -1){
					adjacency[ctr][(ptr[z][x].right)-1] = 1;
				}
				if(ptr[z][x].up != -1){
					adjacency[ctr][(ptr[z][x].up)-1] = 1;
				}
				if(ptr[z][x].down != -1){
					adjacency[ctr][(ptr[z][x].down)-1] = 1;
				}
			ctr++;
			}
		}
	}
	


	//the following loops are for writing to adjacency.txt
	myfile.open ("adjacency.txt");
	myfile << "vertices: 1384 \n";
	myfile << "adjacency matrix: \n";
	for(int m = 0; m<1384; m++){
		for(int b = 0; b<1384; b++){
		myfile <<adjacency[m][b]<<" ";
		}
	myfile << "\n";
	}
	myfile << "\n";
	myfile << "center: \n";

	//this loop also prints the 2d array of the map to the terminal.
	for(int p = 0; p<40; p++){
		for(int l = 0; l<44; l++){
			if(ptr[p][l].value == 0){
			std::cout << "0";
			}
			if(ptr[p][l].value == 100){
			std::cout << "1";
			}
			if(ptr[p][l].value == 0){
				myfile <<ptr[p][l].centerX<<","<<ptr[p][l].centerY<<"\n";
			}
		}
	std::cout << "\n";
	}	
	
	myfile.close();

	callback = 0;
	sub.shutdown();

	//ROS_INFO_STREAM("2darr:"<< twoDarr[0][0] <<  twoDarr[1][1]<<twoDarr[0][439]);
	
} 



int main(int argc,char **argv) {

	srand(time(0));
	ROS_INFO_STREAM("running...");
	//here we initialize and create a node handle
	ros::init(argc,argv,"laser_subscriber");   
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(1000);
	//subscribing to the map data 
	sub = nh.subscribe("/map",1,&scanMessageReceived);
	//this will spin once and go through the callback function once to get the map.
	while(ros::ok() && callback){
	
		ros::spinOnce();

	}

	ROS_INFO_STREAM("DONE");
	return 0;


	

}
