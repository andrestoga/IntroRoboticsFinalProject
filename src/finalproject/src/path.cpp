#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdlib.h>     /* atoi */
#include <queue>          // std::priority_queue
//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <finalproject/path_ser.h>
#include <cmath>        // std::abs

class point{
	public:
	 double x , y;
	 point(){x = 0;; y = 0;}

};
class vertex{
public:
	point pos;
	int bp;
	int index;
	int cost;
	vertex()
	{
		bp = -1; index = -1; cost = 10000;
	}
};

class vtxComp{
	public: 
		bool operator ()(vertex &v1 ,vertex &v2)
		{
			return v1.cost>v2.cost;
		}
};


int *adjmtx;
point * centers_pos;
int vertices_num;

std::vector<vertex> dijkstra(int init, int goal)
{
	bool visited[vertices_num];// = {0};
	for(int i = 0; i < vertices_num; i++){visited[i] = false;}
	std::vector<vertex> closed = std::vector<vertex>();
	std::vector<vertex> path = std::vector<vertex>();
	if(init == goal)
	{
		return path;// init == goal; empty path
	}
	std::priority_queue<vertex ,std::vector<vertex>, vtxComp> pqueue;
	vertex init_v = vertex();
	init_v.bp = -10;// no bp
	init_v.index = init;
	init_v.cost = 0;
	pqueue.push(init_v); //Pushing the source with distance from itself as 0

	while(!pqueue.empty())
	{
		vertex currentVtx = pqueue.top();
		pqueue.pop();
		closed.push_back(currentVtx);

		if(currentVtx.index == goal)
		{
			
			for(int i = closed.size()-1; i >=0 ; i --)
			{
				if(path.size() ==0 && closed[i].index == goal)
				{
					path.push_back(closed[i]);
				}
				else
				{
					if(path.size()>0 && path[path.size() -1].bp == closed[i].index)
					{
						path.push_back(closed[i]);
					}
				}
			}

		for(int i = 0; i < path.size(); i++)
			{
				ROS_INFO_STREAM("index: " << path[i].index <<" bp: " << path[i].bp);
			}
			return path;
		}

		if(visited[currentVtx.index])// because I don't change existing cost, I just add a repeat one
		{
			//ROS_INFO_STREAM("continue");
			continue;
					}
		else
		{
			visited[currentVtx.index] = true;
			// get all reachable vertices
			for(int i = 0; i < vertices_num; i++)
			{
				if(adjmtx[currentVtx.index * vertices_num + i] > 0)
				{
					vertex nextVtx = vertex();
					nextVtx.bp = currentVtx.index;
					nextVtx.cost = currentVtx.cost + adjmtx[currentVtx.index * vertices_num + i];
					nextVtx.index = i;
					//ROS_INFO_STREAM("Next: "<< i);
					pqueue.push(nextVtx);
				}
			}

		}

	}
	return path;

}


int cellIdxOfpoint(double x, double y)
{
	int vtxIdx = -1;
	double min_dist = 100000000;
	for(int i = 0; i < vertices_num; i++)
	{
		//double dist = std::abs(req.pose.position.x - centers_pos[i].x) + std::abs(req.pose.position.y - centers_pos[i].y);
		//ROS_INFO_STREAM("dist: " << dist<<" min_dist:" << min_dist);
		//if(min_dist > dist)
		//{
		//	vtxInit = i;
		//	min_dist = dist;
		//}
		if(std::abs(x - centers_pos[i].x)<=0.5 && std::abs(y - centers_pos[i].y)<=0.5)
		{
			vtxIdx = i;
			break;
		}


		//0.5

	}
	ROS_INFO_STREAM("X: " <<x<<" Y:" << y << "closest: "<< vtxIdx);
	return vtxIdx;
}

bool path_plan_service(finalproject::path_ser::Request & req, finalproject::path_ser::Response & res)
//bool path_plan_service(int & req, bool & res)
{
	//double x = req.init_pose.position.x;
	// find the closest cell
	int vtxInit = cellIdxOfpoint(req.init_pose.position.x,req.init_pose.position.y);
	int vtxGoal = cellIdxOfpoint(req.goal_pose.position.x,req.goal_pose.position.y);

	if(vtxInit == -1)
	{
		geometry_msgs::PoseStamped posd = geometry_msgs::PoseStamped();
		posd.pose.position.x = -1001;// = req.pose;
		posd.pose.position.y = -1001;
		res.path.poses.push_back(posd);
		return true;
	}

	if(vtxGoal == -1)
	{
		geometry_msgs::PoseStamped posd = geometry_msgs::PoseStamped();
		posd.pose.position.x = -1002;// = req.pose;
		posd.pose.position.y = -1002;
		res.path.poses.push_back(posd);
		return true;
	}

	if(vtxInit == vtxGoal)
	{
		geometry_msgs::PoseStamped posd = geometry_msgs::PoseStamped();
		posd.pose.position.x = -1003;// = req.pose;
		posd.pose.position.y = -1003;
		res.path.poses.push_back(posd);
		return true;
	}

	
	geometry_msgs::PoseStamped initPsd = geometry_msgs::PoseStamped();
	initPsd.pose = req.init_pose;
	res.path.poses.push_back(initPsd);
	std::vector<vertex> path = dijkstra(vtxInit, vtxGoal);
	for(int i = path.size() -1; i >=0; i--)
	{
		
		geometry_msgs::PoseStamped posd = geometry_msgs::PoseStamped();
		posd.pose.position.x = centers_pos[path[i].index].x;// = req.pose;
		posd.pose.position.y = centers_pos[path[i].index].y;
		res.path.poses.push_back(posd);
	}

	geometry_msgs::PoseStamped goalPsd = geometry_msgs::PoseStamped();
	goalPsd.pose = req.goal_pose;
	res.path.poses.push_back(goalPsd);


	return true;
}


int main(int argc,char **argv) {
	ros::init(argc,argv,"path");
	ros::NodeHandle nh;

	// write a text file

	/*
	std::ofstream myfile("example.txt", std::ios::out | std::ios::binary);
	myfile << "Writing this to a file.\n";
	myfile.close();*/

	// read a text file
	std::ifstream infile("adjacency.txt");
  	if(!infile.is_open())
	{
    	ROS_INFO_STREAM("file not found!");
    	ros::shutdown();
  	}
  	else
  	{
  		ROS_INFO_STREAM("File found!");
  	}

  	int process_step = 0; // 1 means processing matrix; 2 means processing centers
	std::string line;
	std::getline(infile, line);
	assert(line.find("vertices:") != std::string::npos);
	std::size_t pos_b_num = line.find(":");
	std::string tmpNumStr = line.substr(pos_b_num + 1);
    vertices_num  = atoi (tmpNumStr.c_str());
	ROS_INFO_STREAM("vertices"<< (vertices_num));

	adjmtx = new int[vertices_num * vertices_num];
    int mtx_row = 0;
    centers_pos = new point[vertices_num];

	while (std::getline(infile, line))
	{
	    std::istringstream iss(line);
	    //int a, b;
	    //if (!(iss >> a >> b)) { break; } // error
	    if (!(iss)|| line.find_first_not_of("\t\n\v\f\r") == std::string::npos)
	    { continue; }
	    
	    if(line.find(":") != std::string::npos)
	    {
	    	ROS_INFO_STREAM("Has : inside");
	    	if(line.find("center") != std::string::npos)
	    	{
				process_step = 2;
				mtx_row = 0;
				continue;
	    	}
	    	else if(line.find("adjacency matrix")!= std::string::npos)
	    	{
	    		process_step = 1;
	    		mtx_row = 0;
	    		ROS_INFO_STREAM("Reading adjacency matrix...");
	    		continue;
	    	}
	    }
	    if(process_step == 1)
	    {
	    	for(int j = 0; j < vertices_num; j ++)
	    	{
	    		int tmp = -1;
	    		iss >> tmp;
	    		//ROS_INFO_STREAM(tmp);
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
	   // ROS_INFO_STREAM((const char*)line.c_str());
	    
	    
    
    // process pair (a,b)
	}
	ROS_INFO_STREAM("Ready to plan!");

	ros::ServiceServer path_service = nh.advertiseService("path_plan_service", path_plan_service);

	//for(int i = 0; i < vertices_num; i++)
	//{
	//	ROS_INFO_STREAM("X: "<<centers_pos[i].x<<" Y: "<< centers_pos[i].y);
		//for (int j = 0; j < vertices_num; j++)
		//{
		//	std::cout<<adjmtx[i * vertices_num +j];
		//}
	//}




	    // pub = nh.advertise<geometry_msgs::Twist>(
        //"/cmd_vel", 1000);
    //std::srand(std::time(0));
    //ros::Subscriber sub_est = nh.subscribe("/amcl_pose", 1, & estMsgReced);
	//ros::Subscriber sub = nh.subscribe("/scan", 1, & scanMsgReced);
	ros::spin();


}
