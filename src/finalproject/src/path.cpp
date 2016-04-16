#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
# include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdlib.h>     /* atoi */


class point{
	public:
	 double x , y;
	 point(){x = 0;; y = 0;}

};

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

  	int process_step = 0; // 1 means processing matrix; 2 means processing centers
	std::string line;
	std::getline(infile, line);
	assert(line.find("vertices:") != std::string::npos);
	std::size_t pos_b_num = line.find(":");
	std::string tmpNumStr = line.substr(pos_b_num + 1);
	int vertices_num  = atoi (tmpNumStr.c_str());
	ROS_INFO_STREAM("vertices"<< (vertices_num +1));

	int *adjmtx = new int[vertices_num * vertices_num];
    int mtx_row = 0;
    point * centers_pos = new point[vertices_num];

	while (std::getline(infile, line))
	{
	    std::istringstream iss(line);
	    //int a, b;
	    //if (!(iss >> a >> b)) { break; } // error
	    if (!(iss)|| line.find_first_not_of('\t\n\v\f\r') == std::string::npos)
	    { continue; }
	    
	    if(line.find(":") != std::string::npos)
	    {
	    	ROS_INFO("Has : inside");
	    	if(line.find("centers") != std::string::npos)
	    	{
				process_step = 2;
				mtx_row = 0;
				continue;
	    	}
	    	else if(line.find("adjacency matrix")!= std::string::npos)
	    	{
	    		process_step = 1;
	    		mtx_row = 0;
	    		continue;
	    	}
	    }
	    if(process_step == 1)
	    {
	    	for(int j = 0; j < vertices_num; j ++)
	    	{
	    		int tmp = -1;
	    		iss >> tmp;
	    		ROS_INFO_STREAM(tmp);
	    		adjmtx[mtx_row * vertices_num + j] = tmp;
	    	}
			mtx_row++;
	    }
	    else if(process_step ==2)
	    {
	    	double x = -1, y = -1;
	    	std::size_t split_idx = line.find(",");
	    	std::string num1 = line.substr(0,split_idx);
	    	std::string num2 = line.substr(split_idx+1);
	    	x = atof(num1.c_str());
	    	y = atof(num2.c_str());

	    	centers_pos[mtx_row].x = x;
	    	centers_pos[mtx_row].y = y;
	    	mtx_row++;
	    	//ROS_INFO_STREAM("X: "<<x<<" Y: "<< y);
	    }

	    //ROS_INFO_STREAM(process_step);
	    ROS_INFO((const char*)line.c_str());
	    

    // process pair (a,b)
	}

	for(int i = 0; i < vertices_num; i++)
	{
		ROS_INFO_STREAM("X: "<<centers_pos[i].x<<" Y: "<< centers_pos[i].y);
		//for (int j = 0; j < vertices_num; j++)
		//{
		//	std::cout<<adjmtx[i * vertices_num +j];
		//}
	}




	    // pub = nh.advertise<geometry_msgs::Twist>(
        //"/cmd_vel", 1000);
    //std::srand(std::time(0));
    //ros::Subscriber sub_est = nh.subscribe("/amcl_pose", 1, & estMsgReced);
	//ros::Subscriber sub = nh.subscribe("/scan", 1, & scanMsgReced);
	ros::spin();


}
