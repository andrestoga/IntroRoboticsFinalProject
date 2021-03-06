#include <cstdlib>
#include "ros/ros.h"
#include "finalproject/Navigator.h"
#include "finalproject/PositionRobot.h"
#include "finalproject/random.h"
#include "finalproject/path_ser.h"

#define TRIES 3
#define GOAL_X 0.75
#define GOAL_Y 0.75
#define MAX_SKIP_WAYPOINTS 3
#define MAX_RADIOUS 3.0

int state = 0;

// int justOnce = 1;
int oneReplanning = 1;
int numSkipWaypoints = 1;
int isReplanning = 0;

int isCheckingGoal = 0;

double getDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testingNavClient");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<finalproject::Navigator>("navigation_server");
  ros::ServiceClient client_pos_ser = n.serviceClient<finalproject::PositionRobot>("PositionRobot");
  ros::ServiceClient client_random_ser = n.serviceClient<finalproject::random>("random");
  ros::ServiceClient client_path_ser = n.serviceClient<finalproject::path_ser>("path_plan_service");

  finalproject::Navigator srv;
  std::vector< finalproject::Navigator > srv_vec;

  finalproject::PositionRobot srv_pose;
  srv_pose.request.useless = 1;

  finalproject::random srv_random;
  srv_random.request.time = 15.0;

  finalproject::path_ser srv_path;

  int n_tries = TRIES;
  // int flag = 1;

  /*Testing with fixed waypoints.
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
  srv.request.point.y = 11;*/

  double x, y;

  ros::Rate rate(10);

  // srv_random.request.isRotation = 1;

  // if(client_random_ser.call(srv_random))
  // {
    
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service random");
  //   return 1;
  // }

  while (ros::ok()) /*&& !srv_vec.empty())*/
  {
    //Call server pose to get the position of the robot in order to plan
    if (client_pos_ser.call(srv_pose))
    {
      srv_path.request.init_pose.position.x = x = srv_pose.response.pose.position.x;
      srv_path.request.init_pose.position.y = y = srv_pose.response.pose.position.y;
    }
    else
    {
      ROS_ERROR("Failed to call service pose");
      return 1;
    }

    if (/*1 == state || */0 == state /*&& justOnce*/ || isReplanning)
    {
      // justOnce = 0;
      isReplanning = 0;

      srv_path.request.goal_pose.position.x = GOAL_X;
      srv_path.request.goal_pose.position.y = GOAL_Y;

      srv_path.response.path.poses.clear();
      srv_vec.clear();

      //Planning or re-planning (Getting waypoints calling the path server)
      if (client_path_ser.call(srv_path))
      {
        /*for(int i = 0; i < srv_path.response.path.poses.size(); i++)
        {
          ROS_INFO_STREAM( "Waypoint " << i << " : ("<< srv_path.response.path.poses[i].pose.position.x << ", " << srv_path.response.path.poses[i].pose.position.y<<" )" );
        }*/

          if (srv_path.response.path.poses.size() == 1)
          {
            if ( -1001 == srv_path.response.path.poses[0].pose.position.x )
            {
              ROS_INFO("Init position occupied!!");
              ROS_INFO("X = %f Y = %f", x, y);
            }
            else if( -1002 == srv_path.response.path.poses[0].pose.position.x )
            {
              ROS_INFO("Goal occupied!!");
              return 1;
            }
            else if ( -1003 == srv_path.response.path.poses[0].pose.position.x && isCheckingGoal == 0 )
            {
              ROS_INFO("Init and goal in the same cell!! Good job!!");
              return 1;
            }
            else
            {
              ROS_INFO("Error!!! The code shouldn't arrive here");
              return 1;
            }
          }

          isCheckingGoal = 0;

          for(int i = srv_path.response.path.poses.size() - 1; i >= 0; i--)
          {
          // ROS_INFO_STREAM( "Waypoint " << i << " : ("<< srv_path.response.path.poses[i].pose.position.x << ", " << srv_path.response.path.poses[i].pose.position.y<<" )" );

            srv.request.point.x = srv_path.response.path.poses[i].pose.position.x;
            srv.request.point.y = srv_path.response.path.poses[i].pose.position.y;

            srv_vec.push_back(srv);
          }

          srv = srv_vec.back();
          srv_vec.pop_back();

          ROS_INFO("Trying to reach the goal: %f, %f", srv.request.point.x, srv.request.point.y);

        }
        else
        {
          ROS_ERROR("Failed to call service path");
          return 1;
        }
    }

    if (client.call(srv))
    {
      state = srv.response.info;

      if (2 == state)
      {
        ROS_INFO("Success!!!");
        n_tries = TRIES;

        if (srv_vec.empty())
        {
          srv_random.request.isRotation = 1;

          if(client_random_ser.call(srv_random))
          {
            if (srv_random.response.useless)
            {
              ROS_INFO("Done!!!");
              break;
            }
            else
            {
              isReplanning = 1;
              isCheckingGoal = 1;
            }
          }
          else
          {
            ROS_ERROR("Failed to call service random");
            return 1;
          }
        }

        // ROS_INFO("Trying the next waypoint!!!");

        srv = srv_vec.back();
        srv_vec.pop_back();
        ROS_INFO_STREAM("Remaining waypoints " << srv_vec.size());

        ROS_INFO("Trying to reach the goal: %f, %f", srv.request.point.x, srv.request.point.y);
      }
      else if(1 == state)
      {

        double distance = getDistance(srv_path.request.init_pose.position.x, srv_path.request.init_pose.position.y, GOAL_X, GOAL_Y);

        ROS_INFO("Distance to the goal: %f", distance);

        if (n_tries--)
        {
          ROS_INFO("It fails.....Trying to reach again the waypoint.");
        }
        else
        {          

          if ( ( numSkipWaypoints % MAX_SKIP_WAYPOINTS ) == 0 && distance > MAX_RADIOUS )
          {
            numSkipWaypoints = 1;

            srv_random.request.isRotation = 0;

            //Call the service to move randomly for a few seconds
            if(client_random_ser.call(srv_random))
            {
              ROS_INFO("It fails... replanning");
              isReplanning = 1;
            }
            else
            {
              ROS_ERROR("Failed to call service random");
              return 1;
            }
          }
          else
          {

            ROS_INFO("It fails.....Moving to the next waypoint.");
            ROS_INFO_STREAM("Remaining waypoints " << srv_vec.size());

            if (srv_vec.empty())
            {
              srv_random.request.isRotation = 1;

              if(client_random_ser.call(srv_random))
              {
                if (srv_random.response.useless)
                {
                  ROS_INFO("Done!!!");
                  break;
                }
                else
                {
                  isReplanning = 1;
                  isCheckingGoal = 1;
                }
              }
              else
              {
                ROS_ERROR("Failed to call service random");
                return 1;
              }
            }

            srv = srv_vec.back();
            srv_vec.pop_back();
            numSkipWaypoints++;

            ROS_INFO("Trying to reach the goal: %f, %f", srv.request.point.x, srv.request.point.y);

          }

          n_tries = TRIES;

        }
      }
      else if (0 == state /*&& !justOnce*/)
      {
        ROS_INFO("Robot is lost...");
      // justOnce = 1;
      }
    }
    else
    {
      ROS_ERROR("Failed to call service navigation");
      return 1;
    }
  }//End of while
  return 0;
}