#include "ros/ros.h"
#include "finalproject/Navigator.h"
#include <cstdlib>

#define TRIES 3

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testingNavClient");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<finalproject::Navigator>("navigation_server");
  finalproject::Navigator srv;

  std::vector< finalproject::Navigator > srv_vec;

  srv.request.point.x = 4;
  srv.request.point.y = 14;
  srv_vec.push_back(srv);

  srv.request.point.x = 5;
  srv.request.point.y = 14;
  srv_vec.push_back(srv);

  srv.request.point.x = 6;
  srv.request.point.y = 14;
  srv_vec.push_back(srv);

  srv.request.point.x = 7;
  srv.request.point.y = 14;
  srv_vec.push_back(srv);

  srv.request.point.x = 8;
  srv.request.point.y = 14;
  srv_vec.push_back(srv);





  srv.request.point.x = 9;
  srv.request.point.y = 14;
  srv_vec.push_back(srv);

  srv.request.point.x = 10;
  srv.request.point.y = 14;
  srv_vec.push_back(srv);

  srv.request.point.x = 11;
  srv.request.point.y = 14;
  srv_vec.push_back(srv);

  srv.request.point.x = 11;
  srv.request.point.y = 13;
  srv_vec.push_back(srv);

  srv.request.point.x = 11;
  srv.request.point.y = 12;
  srv_vec.push_back(srv);

  srv.request.point.x = 11;
  srv.request.point.y = 11;
  // srv_vec.push_back(srv);

  ros::Rate rate(10);

  int n_tries = TRIES;
  int flag = 1;

  // srv = srv_vec.back();
  // srv_vec.pop_back();

  while (ros::ok()) /*&& !srv_vec.empty())*/
  {
    if (client.call(srv))
    {
      if (srv.response.isSuccess.data)
      {
        ROS_INFO("Success!!!");

        if (srv_vec.empty())
        {
          ROS_INFO("Done!!!");
          break;
        }

        ROS_INFO("Trying next waypoint!!!");

        srv = srv_vec.back();
        srv_vec.pop_back();
        ROS_INFO_STREAM("Remaining waypoints " << srv_vec.size());
      }
      else
      {
        if (n_tries--)
        {
          ROS_INFO("It fails.....Trying to reach again the waypoint.");
        }
        else
        {
          ROS_INFO("It fails.....Moving to the next point.");
          srv = srv_vec.back();
          srv_vec.pop_back();
          n_tries = TRIES;
        }
      }
    }
    else
    {
      ROS_ERROR("Failed to call service navigation");
      return 1;
    }
  }



  return 0;
}