#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "finalproject/Navigator.h"
#include "finalproject/PositionRobot.h"
#include "std_msgs/Bool.h"
#include <tf/tf.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define MIN_RANGE .4
#define FORWARD_SPEED_ME_PS .3
#define FORWARD_SPEED_M_PS_LOC .1
#define ROTATE_SPEED_DEG_PS 15
#define ROTATE_SPEED_DEG_PS_LOC 10
#define MIN_SCAN_ANGLE_RAD -40.0/180*M_PI
#define TURN_ITERATIONS 6
#define MAX_SCAN_ANGLE_RAD +40.0/180*M_PI
#define MIN_ROT_ANGLE 40
#define MAX_ROT_ANGLE 50
#define RANDOM_ANGLES_RANGE MAX_ROT_ANGLE - MIN_ROT_ANGLE + 1

ros::Publisher vel_Pub;
geometry_msgs::Point currGoal;

//States
//Planning = 0
//Turning = 1
//Forward = 2
int global_state = 0;

int tries = 0;

//States
//Rotate_Goal = 0
//Forward_Goal = 1
int state = 0;
ros::Duration rotateDuration;
ros::Time rotateStartTime;
float theta = 0.0f;
float closestRange;
int alignToGoal = 1;

double t1 = 0.0;
double t2 = 0.0;
int flag = 0;

int once = 1;
float volume = 0.0f;

geometry_msgs::Pose position;
geometry_msgs::Pose pos_gazebo;
double robotAngle = 0.0;
int count = TURN_ITERATIONS;

int justLocalized = 0;

int wrongLocalization = 0;

int isObstacle = 0;

int firstLocalization = 1;

float Calculate_Rotation_Angle(geometry_msgs::Vector3 unitVectorRobot, geometry_msgs::Vector3 unitVectorDirection);
void move(double speed, double distance, int isForward);
bool navigation(finalproject::Navigator::Request  &req,
	finalproject::Navigator::Response &res);
bool positionRobot(finalproject::PositionRobot::Request  &req, finalproject::PositionRobot::Response &res);
void rotate(double angular_speed_rad, double relative_angle, int dir);
double degrees2radians(double angle_in_degrees);
double radians2degress(double angle_in_radians);
void scanMessageReceived(const sensor_msgs::LaserScan msg);
void pose_gazebo_MessageReceived(const geometry_msgs::Pose& pose);
void pose_amcl_MessageReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void stop();
double getDistance(double x1, double y1, double x2, double y2);
void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pos_cov, const geometry_msgs::PoseConstPtr& pos_gaz);
void MoveSlowly(float t, int direction);
void Wait(float t);
void moveRandomly();

int main(int argc,char **argv)
{
	ros::init(argc,argv,"navigation");

	ros::NodeHandle nh;

	ros::Subscriber sub_scan = nh.subscribe("/scan", 10, &scanMessageReceived);
	ros::Subscriber sub_pose_amcl = nh.subscribe("/amcl_pose", 10, &pose_amcl_MessageReceived);
	// ros::Subscriber sub_pose_gazebo = nh.subscribe("/gazebo_pose", 10, &pose_gazebo_MessageReceived);

	// message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub1(nh, "amcl_pose", 1);
 //  	message_filters::Subscriber<geometry_msgs::Pose> sub2(nh, "gazebo_pose", 1);

 //  	typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::Pose> MySyncPolicy;

 //  	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);

  	// sync.registerCallback(boost::bind(&callback, _1, _2));
	
	vel_Pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// Creating a message of type 'geometry_msgs'.
	/*geometry_msgs::Twist msgFor;
	//Assigning the linear and angular velocities.
	msgFor.linear.x = FORWARD_SPEED_ME_PS;
	msgFor.angular.z = 0;

	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgRot;
	//Assigning the linear and angular velocities.
	msgRot.linear.x = 0;
	msgRot.angular.z = degrees2radians(5);

	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgStop;
	//Assigning the linear and angular velocities.
	msgRot.linear.x = 0;
	msgRot.angular.z = 0;*/

	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgRot;
	//Assigning the linear and angular velocities.
	msgRot.linear.x = 0;
	msgRot.angular.z = degrees2radians(5);

	ros::ServiceServer service = nh.advertiseService("navigation_server", navigation);
	ros::ServiceServer service_position = nh.advertiseService("PositionRobot", positionRobot);
	ROS_INFO("Navigating randomly");

	ros::Rate loop_rate(10);

	int toggleDir = 1;

	while (ros::ok())
  	{
  		/*if (1 == global_state)//Turning randomly = 1
  		{
  			//dir = 1 counter clockwise, dir = 0 clockwise
			rotate( degrees2radians(3), degrees2radians(rand() % RANDOM_ANGLES_RANGE + MIN_ROT_ANGLE), 0);
  		}*/
  		if (1 == global_state)//Forward until reach and obstacle = 2
  		{
			// vel_Pub.publish(msgFor);

			if ( justLocalized )
			{
				justLocalized = 0;
				ROS_INFO("Robot Localized");
			}
  		}
  		else if (0 == global_state)
  		{
			justLocalized = 1;

  			ROS_INFO("Trying to localize the robot.");

  			rotate( degrees2radians(ROTATE_SPEED_DEG_PS_LOC), degrees2radians(rand() % RANDOM_ANGLES_RANGE + MIN_ROT_ANGLE), 1);

  			// moveRandomly();

  	// 		if (toggleDir)
  	// 		{
  	// 			MoveSlowly(1.0, toggleDir);
  	// 			toggleDir = 0;
  	// 		}
  	// 		else
  	// 		{
  	// 			MoveSlowly(1.0, toggleDir);
  	// 			toggleDir = 1;
  	// 		}

			// // MoveSlowly(1.0);
			// Wait(3.0);

  			// if (firstLocalization)
  			// {
  			// 	firstLocalization = 1;
  			// }
  			// else
  			// {
  			// 	rotate( degrees2radians(ROTATE_SPEED_DEG_PS_LOC), degrees2radians(rand() % RANDOM_ANGLES_RANGE + MIN_ROT_ANGLE), 1);
  			// }

			// vel_Pub.publish(msgRot);

  		}

 //  		ROS_INFO("Global state: %d", global_state);

  		ros::spinOnce();
    	loop_rate.sleep();
  	}
}

void moveRandomly()
{
	geometry_msgs::Twist msgFor;
	//Assigning the linear and angular velocities.
	msgFor.linear.x = FORWARD_SPEED_M_PS_LOC;
	msgFor.angular.z = 0;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		if (1 == isObstacle)
		{
			rotate( degrees2radians(ROTATE_SPEED_DEG_PS_LOC), degrees2radians(rand() % RANDOM_ANGLES_RANGE + MIN_ROT_ANGLE), 1);
		}
		else
		{
			vel_Pub.publish(msgFor);
		}

		if (1 == global_state)
		{
			stop();
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pos_cov, const geometry_msgs::PoseConstPtr& pos_gaz)
{

}

bool positionRobot(finalproject::PositionRobot::Request  &req, finalproject::PositionRobot::Response &res)
{
	res.pose.position.x = position.position.x;
	res.pose.position.y = position.position.y;
	// res.pose.position.z = position.position.z;

	// res.pose.orientation.x = position.orientation.x;
	// res.pose.orientation.y = position.orientation.y;
	// res.pose.orientation.z = position.orientation.z;
	// res.pose.orientation.w = position.orientation.w;

	return true;
}

void stop()
{
	geometry_msgs::Twist vel_msg;

	vel_msg.linear.x =0;
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

	/*if (once)
	{
		t1 = ros::Time::now().toSec();
		once = 0;
	}

	t2 = ros::Time::now().toSec();*/

	//Completly localized
	//If the sum of the elements of the diagonal shrink below to 0.1
	//Completly lost if the sum of the elements of the diagonal are greater than 2.0

	/*if ((t2 - t1) > 5.0)
	{
		flag = 1;
	}*/

	// ROS_INFO("Covariance value: %f  %f  %d", volume, t2 - t1, flag);
	ROS_INFO("Covariance value: %f", volume);
	// ROS_INFO("Pos Gazebo: %f %f", pos_gazebo.position.x, pos_gazebo.position.y);

	/*if (flag)
	{*/
		// ROS_INFO("AMCL callback!!!!");

		if ( volume < 0.1 && !wrongLocalization )
		{
			position.position.x = msg->pose.pose.position.x;
			position.position.y = msg->pose.pose.position.y;
			position.position.z = msg->pose.pose.position.z;

			position.orientation.x = msg->pose.pose.orientation.x;
			position.orientation.y = msg->pose.pose.orientation.y;
			position.orientation.z = msg->pose.pose.orientation.z;
			position.orientation.w = msg->pose.pose.orientation.w;

			tf::Quaternion q;
			tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

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

			global_state = 1;

			// ROS_INFO("AMCL: %f %f", position.position.x, position.position.y);
			// ROS_INFO("Robot is localized!!!");
		}
		else
		{
			// ROS_INFO("Robot is lost!!!");
			// ROS_INFO("Trying to localize itself again...");
			// ROS_INFO("Not localized: %f %f", position.position.x, position.position.y);
			global_state = 0;
		}

		// if (volume > 2.0)
		// {
		// 	ROS_INFO("Robot is lost!!!");
		// 	ROS_INFO("Trying to localize itself again...");
		// 	global_state = 2;
		// }
	// }
}

void pose_gazebo_MessageReceived(const geometry_msgs::Pose& pose)
{
	// ROS_INFO("Gazebo: %f %f", pose.position.x, pose.position.y);

	pos_gazebo.position.x = pose.position.x;
	pos_gazebo.position.y = pose.position.y;

	// position.position.x = pose.position.x;
	// position.position.y = pose.position.y;
	// position.position.z = pose.position.z;

	// tf::Quaternion q;
	// tf::quaternionMsgToTF(pose.orientation, q);

 //    // the tf::Quaternion has a method to acess roll pitch and yaw
	// double roll, pitch, yaw;
	// tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	// if (yaw < 0)
	// {
	// 	robotAngle = 2 * M_PI + yaw;
	// }
	// else
	// {
	// 	robotAngle = yaw;
	// }
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
				// ROS_INFO("Closest range %f", closestRange);
			}
		}
	}

	// ROS_INFO_STREAM("Closest range: " << closestRange);

	// ROS_INFO_STREAM("Min angle: " << msg.angle_min*(180/M_PI) << "\nMax angle" << msg.angle_max*(180/M_PI));

	// If it is less than the allowed range, rotate to a random direction.
	if(closestRange < MIN_RANGE)
	{
		/*if (global_state != 0)
		{*/
			// global_state = 2;
			isObstacle = 1;
		// ROS_INFO("Stop motherfucker!!!");

		// }
		
	}
	else
	{
		// if (global_state != 0)
		// {
		// 	global_state = 1;
		// }

		isObstacle = 0;	
		// ROS_INFO("OK");
	}
}

double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees *M_PI /180.0;
}

double radians2degress(double angle_in_radians)
{
	return angle_in_radians * 180.0/M_PI;	
}

//dir = 1 counter clockwise, dir = 0 clockwise
void rotate(double angular_speed_rad, double relative_angle, int dir)
{
	geometry_msgs::Twist vel_msg;

   //set a random linear velocity in the x-axis
	vel_msg.linear.x = 0.0;
	vel_msg.linear.y = 0.0;
	vel_msg.linear.z = 0.0;
   //set a random angular velocity in the y-axis
	vel_msg.angular.x = 0.0;
	vel_msg.angular.y = 0.0;

	if (dir)
	{
		vel_msg.angular.z = std::abs(angular_speed_rad);
	}
	else
	{
		vel_msg.angular.z = -std::abs(angular_speed_rad);
	}

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(10);

   // ROS_INFO_STREAM("Angular speed" << angular_speed << std::endl);

	do{
		vel_Pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed_rad * (t1-t0);

		// if ( 0 == global_state )
		// {
		// 	break;
		// }

		ros::spinOnce();
		loop_rate.sleep();
	   // cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<std::endl;
		// ROS_INFO_STREAM((t1-t0)<<", "<<current_angle <<", "<<relative_angle << ", " << vel_msg.angular.z <<", "<< angular_speed << std::endl);
	}while(current_angle < relative_angle);

	vel_msg.angular.z =0;
	vel_Pub.publish(vel_msg);
}

void MoveSlowly(float t, int direction)
{
	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgFor;
	//Assigning the linear and angular velocities.
	// msgFor.linear.x = FORWARD_SPEED_M_PS_LOC;

	stop();

	if (direction)
	{
		msgFor.linear.x = std::abs(FORWARD_SPEED_M_PS_LOC);
	}
	else
	{
		msgFor.linear.x = -std::abs(FORWARD_SPEED_M_PS_LOC);
	}

	double tt1 = 0.0;
	double tt2 = 0.0;

	tt1 = ros::Time::now().toSec();

	while ((tt2 - tt1) < t)
	{
		vel_Pub.publish(msgFor);
  		ros::spinOnce();

		if ( 1 == isObstacle )
		{
			stop();
			break;
		}  		

  		tt2 = ros::Time::now().toSec();
  	}

  	if ( 1 == isObstacle )
	{
		ROS_INFO("Rotating Localizing");
		rotate( degrees2radians(ROTATE_SPEED_DEG_PS_LOC), degrees2radians(rand() % RANDOM_ANGLES_RANGE + MIN_ROT_ANGLE), 1);
	}

	stop();
}

void Wait(float t)
{
	// Creating a message of type 'geometry_msgs'.
	geometry_msgs::Twist msgFor;
	//Assigning the linear and angular velocities.
	msgFor.linear.x = 0;
	msgFor.angular.z = 0;

	double tt1 = 0.0;
	double tt2 = 0.0;

	tt1 = ros::Time::now().toSec();

	while ((tt2 - tt1) < t)
	{
		vel_Pub.publish(msgFor);
  		ros::spinOnce();
  		tt2 = ros::Time::now().toSec();
  	}
}

bool navigation(finalproject::Navigator::Request  &req,
	finalproject::Navigator::Response &res)
{
	double distance = 0.0;

	if ( -1001 == req.point.x)
	{
		wrongLocalization = 1;
		global_state = 0;
	}
	else
	{
		wrongLocalization = 0;	
	}

	distance = getDistance(req.point.x, req.point.y, position.position.x, position.position.y);

	// if (distance > 4.0)
	// {
	// 	res.info = 0;
	// 	wrongLocalization = 1;
	// 	global_state = 0;

	// 	ROS_INFO("Distance too large. Robot is not localized correctly. Distance: %f", distance);
	// }

	if ( 1 == global_state  && !wrongLocalization)
	{
		ros::Rate rate(10);
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
						goalVec.x = req.point.x - position.position.x;
						goalVec.y = req.point.y - position.position.y;

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

						if ( 0 == global_state)
						{
							Wait(5.0);
						}

						float degreesToRotate = radians2degress(theta);

						// if (degreesToRotate > 80.0 || tries < 3)
						// {
						// 	res.info = 2;
						// 	tries++;
						// 	isExit = 1;
						// 	break;
						// }

						ROS_INFO("Rotating towards a waypoint %f", degreesToRotate);

						rotate( degrees2radians(ROTATE_SPEED_DEG_PS), theta, dir);
						Wait(1.0);

					}
					else
					{
						count = TURN_ITERATIONS;
						state = 1;
						Wait(1.0);

					}		

				break;

				case 1:

					// if (distance > 4.0)
					// {
					// 	res.info = 0;
					// 	wrongLocalization = 1;
					// 	ROS_INFO("Distance too large. Robot is not localized correctly. Distance: %f", distance);
					// }
					// else
					// {
						wrongLocalization = 0;
						ROS_INFO("Moving towards a waypoint,");

						move(FORWARD_SPEED_ME_PS, distance, 1);

						if ( 1 == isObstacle )
						{
							//TODO: Count the number of attemps to reach the goal
							ROS_INFO("Obstacle found during the path!!!");
							res.info = 1;
							// rotate( degrees2radians(ROTATE_SPEED_DEG_PS), degrees2radians(rand() % RANDOM_ANGLES_RANGE + MIN_ROT_ANGLE), 0;
							// global_state = 0;
						}
						else
						{
							//Reached to the goal!!
							ROS_INFO("Goal reached!!\nFinal position:\nX: %f\nY: %f", position.position.x, position.position.y);
							res.info = 2;
						}
					// }

					state = 0;
					isExit = 1;

					Wait(1.0);

					// if ( alignToGoal )
					// {
					// 	state = 0;
					// 	alignToGoal = 0;
					// 	ROS_INFO("Aligning to the goal\nX: %f\nY: %f", position.x, position.y);
					// }
					// else
					// {
						
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

	}
	else if( global_state == 0 || wrongLocalization)//Lost!!
	{
		res.info = 0;
	}

	return true;	
}//End of function navigation

void move(double speed, double distance, int isForward)
{
	stop();

	geometry_msgs::Twist vel_msg;

   //set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x = std::abs(speed);
	else
		vel_msg.linear.x = -std::abs(speed);

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(10);

	do{
		vel_Pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		//TODO: Replaced with the position of the robot from the /amcl_pose
		current_distance = speed * (t1-t0);

		// ROS_INFO("Current position: x = %f, y = %f\n", position.position.x, position.position.y);
		// ROS_INFO("Distance to the goal: %f\nCurrent goal: %f %f", getDistance(currGoal.x, currGoal.y, position.position.x, position.position.y), currGoal.x, currGoal.y);

		ros::spinOnce();

		if ( 1 == isObstacle)
		{
			break;
		}

		loop_rate.sleep();
	   //cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);

	stop();
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