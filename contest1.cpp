#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)

float angular = 0.0;
float linear = 0.0;
float posX=0.0, posY=0.0, yaw=0.0;
uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist=std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=15;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    bumper[msg->bumper]=msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);

    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    ROS_INFO("Min dist: %f", minLaserDist);


}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f deg", posX, posY, yaw, RAD2DEG(yaw));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    int state=0;
    uint64_t turnStart=0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        //create an array that stores the robot's position every 5 seconds x position in column 1 and y position in column 2
        //this is a start for position tracking can think of a way to use this array to recognize when we are in a loop
        int positionArray[96][2];
        //I am not convinced this will work might have coded it wrong :/ 
        for (int i = 0; i<96; ++i) {
            if secondsElapsed % 5 == 0 {
                positionArray[i][1] = posX;
                positionArray[i][2] = posY;
            }
        }

        //start by doing a 360 scan of the area (can make this a function instead and call on it multiple times)
        turnStart = secondsElapsed;
        while (secondsElapsed - turnStart <= 12) {
            angular = M_PI/6; //slow turn speed to ensure mapping and sensing is accurate
            linear = 0.0;
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
        }


        //check if any of the bumpers are pressed
        bool any_bumper_pressed=false;
        for (uint32_t b_idx=0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }

        if (any_bumper_pressed) {
            //insert bumper function from Stephanie here for robot to react accordingly
        }
        else if(minLaserDist>1.0 && minLaserDist!=std::numeric_limits<float>::infinity()){
            //if there is a free path greater than 1.0 m, travel forward at 0.25 m/s
            linear=0.25;
            angular=0.0;
            state=0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
        } 
        else if (minLaserDist<1.0 && minLaserDist>0.5 && minLaserDist!=std::numeric_limits<float>::infinity()){
            //if you are within 1.0 m of an obstacle, travel forward at 0.1 m/s
            linear=0.1;
            angular=0.0;
            state=0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
        }
        else if (minLaserDist<0.5 || minLaserDist==std::numeric_limits<float>::infinity()){
            if (/*corridor or corner == TRUE*/) {
                //use Kevin's code to react accordingly
            }
            else {
                //can update this with Tiger's turning and decision making function
                
                //if you are within 0.5 m of an obstacle, stop moving and turn until there is no more obstacle within 0.5 m of the scanner
                //for first 4 minutes turn left when running into obstacle
                while((minLaserDist<0.5 || minLaserDist==std::numeric_limits<float>::infinity()) && secondsElapsed <= 240) {
                angular=M_PI/9;
                linear=0.0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                }
                //for last 4 minutes turn right when running into obstacle
                while((minLaserDist<0.5 || minLaserDist==std::numeric_limits<float>::infinity()) && secondsElapsed > 240) {
                angular=-M_PI/9;
                linear=0.0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                }
            }
        }
        
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    //add in line to print or save map if that's something done in the code otherwise delete this line

    return 0;
}

float displaceYawTime(float yawDispDeg, float angularVel){
    float yawDispRad=DEG2RAD(yawDispDeg);
    float timeToTurn=yawDispRad/angularVel;
    return timeToTurn;
}