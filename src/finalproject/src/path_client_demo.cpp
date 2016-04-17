#include <ros/ros.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Path.h>
#include <finalproject/path_ser.h>

int main(int argc,char **argv) {
	ros::init(argc,argv,"path_client_demo");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<finalproject::path_ser>("path_plan_service");
    finalproject::path_ser srv;
    srv.request.pose.position.x = 11.5;
    srv.request.pose.position.y = 11.5;
    if (client.call(srv))
    {
    	for(int i = 0; i < srv.response.path.poses.size(); i++)
    	{
    		ROS_INFO_STREAM("Waypoint "<<i<<" : ("<< srv.response.path.poses[i].pose.position.x<<", "<< srv.response.path.poses[i].pose.position.y<<" )");
    	}
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }
  
    return 0;
}




