#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include "finalproject/Navigator.h"
#include "std_msgs/Bool.h"
#include <tf/tf.h>

#define MIN_RANGE 1
#define FORWARD_SPEED_MPS .2
#define ROTATE_SPEED_ANGPS 10
#define ROTATE_SPEED_RADPS M_PI/18
#define MIN_SCAN_ANGLE_RAD -40.0/180*M_PI
#define TURN_ITERATIONS 8
#define MAX_SCAN_ANGLE_RAD +40.0/180*M_PI

ros::Publisher vel_Pub;
geometry_msgs::Point currGoal;

// States
//Rotate_Goal = 0
//Forward_Goal = 1
int state = 0;
ros::Duration rotateDuration;
ros::Time rotateStartTime;
float theta = 0.0f;
int closestRange;
int alignToGoal = 1;

double t1 = 0.0;
double t2 = 0.0;
int flag = 0;

int once = 1;
float volume = 0.0f;

geometry_msgs::Vector3 position;
double robotAngle = 0.0;
int count = TURN_ITERATIONS;

float Calculate_Rotation_Angle(geometry_msgs::Vector3 unitVectorRobot, geometry_msgs::Vector3 unitVectorDirection);
void move(double speed, double distance, int isForward);
bool navigation(finalproject::Navigator::Request  &req,
	finalproject::Navigator::Response &res);
void rotate(double angular_speed, double relative_angle, int dir);
double degrees2radians(double angle_in_degrees);
void scanMessageReceived(const sensor_msgs::LaserScan msg);
void pose_gazebo_MessageReceived(const geometry_msgs::Pose& pose);
void pose_amcl_MessageReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void stop();
double getDistance(double x1, double y1, double x2, double y2);

int main(int argc,char **argv)
{
	ros::init(argc,argv,"navigation");

	ros::NodeHandle nh;

	ros::Subscriber sub_scan = nh.subscribe("/scan", 10, &scanMessageReceived);
	// ros::Subscriber sub_pose_amcl = nh.subscribe("/amcl_pose", 10, &pose_amcl_MessageReceived);
	ros::Subscriber sub_pose_gazebo = nh.subscribe("/gazebo_pose", 10, &pose_gazebo_MessageReceived);
	
	vel_Pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgFor;
	//Assigning the linear and angular velocities.
	msgFor.linear.x = FORWARD_SPEED_MPS;
	msgFor.angular.z = 0;

	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgRot;
	//Assigning the linear and angular velocities.
	msgRot.linear.x = 0;
	msgRot.angular.z = ROTATE_SPEED_RADPS;

	ros::ServiceServer service = nh.advertiseService("navigation_server", navigation);
	ROS_INFO("Ready to navigate");
	ros::spin();
}

void stop()
{
	geometry_msgs::Twist vel_msg;

	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	vel_Pub.publish(vel_msg);
}

void pose_amcl_MessageReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{


	volume = msg->pose.covariance[0] + msg->pose.covariance[7] + msg->pose.covariance[14] + msg->pose.covariance[21] + msg->pose.covariance[28] + msg->pose.covariance[35];

	if (once)
	{
		t1 = ros::Time::now().toSec();
		once = 0;
	}

	t2 = ros::Time::now().toSec();

	//Completly localized
	//If the sum of the elements of the diagonal shrink below to 0.1
	//Completly lost if the sum of the elements of the diagonal are greater than 2.0

	if ((t2 - t1) > 5.0)
	{
		flag = 1;
	}

	ROS_INFO("Covariance value: %f  %f  %d", volume, t2 - t1, flag);

	if (flag)
	{
		if (volume < 0.1)
		{
			state = 2;
			ROS_INFO("Robot is localized!!!");
		}

		if (volume > 2.0)
		{
			ROS_INFO("Robot is lost!!!");
			state = 2;
		}
	}
}

void pose_gazebo_MessageReceived(const geometry_msgs::Pose& pose)
{
	position.x = pose.position.x;
	position.y = pose.position.y;
	position.z = pose.position.z;

	tf::Quaternion q;
	tf::quaternionMsgToTF(pose.orientation, q);

    // the tf::Quaternion has a method to acess roll pitch and yaw
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	if (yaw < 0)
	{
		robotAngle = 2 * M_PI + yaw;
	}
	else
	{
		robotAngle = yaw;
	}
}

void scanMessageReceived(const sensor_msgs::LaserScan msg)
{
	float min = msg.range_min;
	float max = msg.range_max;
	// ROS_INFO_STREAM("Laser configuration");
	// ROS_INFO_STREAM("Min Range " << min << " Max Range " << max);
	// ROS_INFO_STREAM("Ranges");

	//TODO: Limit the range of the lidar sensor.
	//Converting min and max radians to steps in order to iterate over the array.
	unsigned int minIndex = ceil((MIN_SCAN_ANGLE_RAD - msg.angle_min) / msg.angle_increment);
	unsigned int maxIndex = ceil((MAX_SCAN_ANGLE_RAD - msg.angle_min) / msg.angle_increment);

  	//Setting initial value to closest range.
	closestRange = msg.ranges[minIndex];

	for (int i = minIndex + 1; i < maxIndex; i++ )
	{
  		//Gathering the closest range.
		if (msg.ranges[i] >= min && msg.ranges[i] <= max)
		{
			if (msg.ranges[i] < closestRange)
			{
				closestRange = msg.ranges[i];
			}
		}
	}

	// ROS_INFO_STREAM("Closest range: " << closestRange);

	// ROS_INFO_STREAM("Min angle: " << msg.angle_min*(180/M_PI) << "\nMax angle" << msg.angle_max*(180/M_PI));

	// If it is less than the allowed range, rotate to a random direction.
	if(closestRange < MIN_RANGE)
	{
		if (state != 2)
		{
			state = 1;	
		}
	}
	else
	{
		if (state != 2)
		{
			state = 0;
		}
	}
}

double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees *M_PI /180.0;
}

void rotate(double angular_speed, double relative_angle, int dir)
{
	geometry_msgs::Twist vel_msg;

   //set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (dir)
	{
		vel_msg.angular.z = std::abs(angular_speed);
	}
	else
	{
		vel_msg.angular.z = -std::abs(angular_speed);
	}

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(10);

   // ROS_INFO_STREAM("Angular speed" << angular_speed << std::endl);

	do{
		vel_Pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	   // cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<std::endl;
		// ROS_INFO_STREAM((t1-t0)<<", "<<current_angle <<", "<<relative_angle << ", " << vel_msg.angular.z <<", "<< angular_speed << std::endl);
	}while(current_angle < relative_angle);

	vel_msg.angular.z =0;
	vel_Pub.publish(vel_msg);
}

bool navigation(finalproject::Navigator::Request  &req,
	finalproject::Navigator::Response &res)
{
	ros::Rate rate(10);
	double distance = 0.0;
	int dir;
	int isExit = 0;

	currGoal.x = req.point.x;
	currGoal.y = req.point.y;

	while (ros::ok())
	{
		geometry_msgs::Vector3 unitVectorRobot;
		dir = 1;

		unitVectorRobot.x = cos(robotAngle);
		unitVectorRobot.y = sin(robotAngle);

		switch(state)
		{
			case 0:

				if(count--)
				{
					//Create a vector for the goal
					geometry_msgs::Vector3 goalVec;
					goalVec.x = req.point.x - position.x;
					goalVec.y = req.point.y - position.y;

					// goalVec = convertToUnitVector(goalVec);

					//Convert the vector goal to unit
					double norm = sqrt(pow(goalVec.x, 2) + pow(goalVec.y, 2));
					goalVec.x /= norm;
					goalVec.y /= norm;

					theta = Calculate_Rotation_Angle(unitVectorRobot, goalVec);

					if (theta < 0)
					{
						dir = 0;
						theta *= -1;
					}
					else
					{
						dir = 1;
					}

					rotate( degrees2radians(ROTATE_SPEED_ANGPS), theta, dir);
				}
				else
				{
					count = TURN_ITERATIONS;
					state = 1;
				}		

			break;

			case 1:

				distance = getDistance(req.point.x, req.point.y, position.x, position.y);

				move(FORWARD_SPEED_MPS, distance, 1);

				stop();

				// if ( alignToGoal )
				// {
				// 	state = 0;
				// 	alignToGoal = 0;
				// 	ROS_INFO("Aligning to the goal\nX: %f\nY: %f", position.x, position.y);
				// }
				// else
				// {
					//Reached to the goal!!
					ROS_INFO("Goal reached!!\nFinal position:\nX: %f\nY: %f", position.x, position.y);
					res.isSuccess.data = true;
					state = 0;
					isExit = 1;
				// }

			break;

			default:
				ROS_INFO("Error. Code shouldn't arrive here...");

		}//End of switch

		// if (!alignToGoal && isExit)
		// {
		// 	alignToGoal = 1;
		// 	break;
		// }

		if (isExit)
		{
			break;
		}

		ros::spinOnce();//Call this function to process ROS incoming messages.

		rate.sleep();//Sleep the rest of the cycle until 10 Hz

	}//End of while

	state = 0;

	return true;
}//End of function navigation

void move(double speed, double distance, int isForward)
{
	geometry_msgs::Twist vel_msg;

   //set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x = std::abs(speed);
	else
		vel_msg.linear.x = -std::abs(speed);

	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(10);

	do{
		vel_Pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);

		ROS_INFO("Current position: x = %f, y = %f\n", position.x, position.y);
		ROS_INFO("Distance to the goal: %f\nCurrent goal: %f %f", getDistance(currGoal.x, currGoal.y, position.x, position.y), currGoal.x, currGoal.y);

		// if (obstacle)
		// {
		// 	stop();
		// 	return;
		// }

		ros::spinOnce();
		loop_rate.sleep();
	   //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);

	vel_msg.linear.x =0;
	vel_Pub.publish(vel_msg);
}

float Calculate_Rotation_Angle(geometry_msgs::Vector3 unitVectorRobot, geometry_msgs::Vector3 unitVectorDirection)
{
	float angle = 0.0;
	float dot = 0.0;
	float det = 0.0;

	// ROS_INFO("Angle of rotation: %f", robotAngle);
	// ROS_INFO("X = %f Y = %f", robotAngleVec.x, robotAngleVec.y);
	// ROS_INFO("X = %f Y = %f", goalVec.x, goalVec.y);

	//Calculate and return the angle between the two vectors
	dot = unitVectorRobot.x * unitVectorDirection.x + unitVectorRobot.y * unitVectorDirection.y;//dot product
	det = unitVectorRobot.x * unitVectorDirection.y - unitVectorRobot.y * unitVectorDirection.x;//determinant
	angle = atan2(det, dot);//atan2(y, x) or atan2(sin, cos)

	return angle;
}

double getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}