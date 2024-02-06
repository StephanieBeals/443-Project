#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <random>

#include <cmath>
#include <iostream>
#include <chrono>

enum Bumper
{
	Left = 0,
	Center = 1,
	Right = 2,
};

#define LIN_SPEED 0.25

using namespace std;

std::random_device rd;
std::mt19937 gen(rd());
std::bernoulli_distribution bernoulli(0.7);

bool turnLeft = true;

Bumper whichBumperHit = Center;

// CALLBACK VARIABLES
bool bumperLeft = false, bumperCenter = false, bumperRight = false;
bool isBumperHit = false;
bool spin = false;

// Odometry
bool isTurning = false;
double posX, posX_Init = 0.0, posX_Spin = 0.0;
double posY, posY_Init = 0.0, posY_Spin = 0.0; 
double yaw, yawInitial, yawDesired;
double pi = 3.1416;

// Laser Scan
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 50;



void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	if (msg->bumper == 0)
	{
		bumperLeft = !bumperLeft;
	}
	else if (msg->bumper == 1)
	{
		bumperCenter = !bumperCenter;
	}
	else if (msg->bumper == 2)
	{
		bumperRight = !bumperRight;
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle * pi / (180.0 * msg->angle_increment);
	laserRange = 11;

	if (desiredAngle * pi / 180.0 < msg->angle_max && -desiredAngle * pi / 180.0 > msg->angle_min)
	{
		for (int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; ++i)
		{
			if (laserRange > msg->ranges[i])
			{
				laserRange = msg->ranges[i];
			}
		}
	}
	else
	{
		for (int i = 0; i < laserSize; ++i)
		{
			if (laserRange > msg->ranges[i])
			{
				laserRange = msg->ranges[i];
			}
		}
	}

	if (laserRange == 11)
	{
		laserRange = 0;
	}

	ROS_INFO("Distance Reading: %lf", laserRange);
	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	if (isTurning && std::abs(yaw - yawInitial) > pi / 2.5)
	{
		isTurning = false;
		turnLeft = bernoulli(gen) ? true : false;
	}

	// ROS_INFO("YAW_DIFF: %lf isTurning: %d", std::abs(yaw - yawInitial), isTurning);
	ROS_INFO("YAW: %lf", yaw);
}

bool isBumperPressed()
{
	return bumperRight || bumperCenter || bumperLeft;
}

double distFromInit()
{
	std::sqrt(std::pow(posX - posX_Init, 2) + std::pow(posY - posY_Init, 2));
}

double distFromLastSpin()
{
	std::sqrt(std::pow(posX - posX_Spin, 2) + std::pow(posY - posY_Spin, 2));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	// Subscritpions
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);

	// Publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	geometry_msgs::Twist vel;
	
	double linear = 0.0;
	double angular = 0.0;

	std::chrono::time_point<std::chrono::system_clock> start;
	start = std::chrono::system_clock::now(); /* start timer */
    	uint64_t secondsElapsed = 0; // the timer just started, so we know it is less than 480, no need to check.
	while(ros::ok() && secondsElapsed <= 480)
	{
		// UPDATE VALUES IN CALLBACK SUBSCRIPTION
		ros::spinOnce();	

		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		angular = 0.0;
		linear = LIN_SPEED;

		if (!isBumperPressed())
		{
			if (laserRange < 0.5 && !isTurning)
			{
				yawInitial = yaw;
				isTurning = true;
			}

			if (laserRange > 0.5 && laserRange < 0.7)
			{
				linear = 0.8 * LIN_SPEED;	
			}

			double dist = distFromLastSpin();
			if (!spin && dist > 0.6)
			{
				ROS_INFO("yawDesired: %lf, yaw: %lf", yawDesired, yaw); 
				if (yaw < 0)
				{
					yawDesired = yaw + 2*pi;
				}
				else
				{
					yawDesired = yaw;
				}

				//yawDesired -= pi/4;
				if (yawDesired < 0)
				{
					yawDesired += 2*pi;
				}
				spin = true;
			}
		}
		else
		{
			posX_Init = posX;
			posY_Init = posY;
			isBumperHit = true;

			if (bumperRight)
			{
				whichBumperHit = Bumper::Right;
			}
			else if (bumperLeft)
			{
				whichBumperHit = Bumper::Left;
			}
			else
			{
				whichBumperHit = Bumper::Center;
			}
			
		}

		if (isBumperHit)
		{
			if (distFromInit() < 0.2)
			{
				switch(whichBumperHit)
				{
					case Bumper::Right:
						angular = pi/6;
						break;
					case Bumper::Left:
						angular = -pi/6;
						break;
					default:
						angular = turnLeft ? pi/6 : -pi/6;
				}

				linear = -LIN_SPEED;
			}
			else
			{
				isBumperHit = false;
			}
		}

		if (isTurning && turnLeft)
		{
			angular = pi/6;
			linear = 0.0;	
		}
		else if (isTurning && !turnLeft)
		{
			angular = -pi/6;
			linear = 0.0;
		}

		if (spin)
		{
			angular = pi/6;
			linear = 0.0;
			
			// Stop Condition
			float yawWack = yaw;
			if (yawWack < 0)
			{
				yawWack = 2*pi + yawWack;
			}
			//ROS_INFO("lower bound: %lf, yaw: %lf, upper bound: %lf", yawDesired-0.5, yawWack, yawDesired);
			if (yawWack > yawDesired-pi/18 && yawWack < yawDesired)
			{
				spin = false;
				posX_Spin = posX;
				posY_Spin = posY;
			}
		}
		
  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);

		// The last thing to do is to update the timer.
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
	}

	return 0;
}
