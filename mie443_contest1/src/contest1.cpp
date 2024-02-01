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
#include <thread>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI/180.)

using Clock = std::chrono::system_clock;

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
//Go to https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
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

void displaceYaw(int yawDispDeg);



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(100);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    int state=0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        auto now =  std::chrono::system_clock::now();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();
        ROS_INFO("Turn start: %lu", millis);
        //displaceYaw(90);
        float yawDispRad=DEG2RAD(95);
        float angularVel=M_PI/6; //30 deg per sec
        float timeToTurn=yawDispRad/angularVel;
        std::chrono::time_point<std::chrono::system_clock> start;
        start = std::chrono::system_clock::now();
    //ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
        ros::Publisher yaw_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
        geometry_msgs::Twist vel;
        
        auto yawStart =  std::chrono::system_clock::now();
        uint64_t millisElapsed=0;
        while(ros::ok() && millisElapsed<=timeToTurn*1000){
            ros::spinOnce();
            vel.angular.z=angularVel;
            vel.linear.x=linear;
            yaw_pub.publish(vel);

            millisElapsed=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-yawStart).count();
        }
        now =  std::chrono::system_clock::now();
        millis = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();
        ROS_INFO("Turn end: %lu", millis);

        /*
        if(minLaserDist>0.75 && minLaserDist!=std::numeric_limits<float>::infinity()){
            linear=0.4;
            angular=0.0;
            state=0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
        } else if (minLaserDist<0.75 || minLaserDist==std::numeric_limits<float>::infinity()){
            turnStart=secondsElapsed; /*
            while(secondsElapsed-turnStart<=3){
                angular=M_PI/6;
                linear=0.0;
                secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }
        }
        */
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep(); //Why is this here?
    }

    return 0;
}
//Use a callback instead???
/*
void displaceYaw(int yawDispDeg){ 
    //Need some code handling negative or greater than 360 case.
    float yawDispRad=DEG2RAD(yawDispDeg);
    float angularVel=M_PI/6; //30 deg per sec
    float timeToTurn=yawDispRad/angularVel;
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
  //ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Publisher yaw_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    
    auto yawStart =  std::chrono::system_clock::now();
    uint64_t millisElapsed=0;
    while(ros::ok() && millisElapsed<=timeToTurn*1000){
        ros::spinOnce();
        vel.angular.z=angularVel;
        vel.linear.x=linear;
        yaw_pub.publish(vel);

        millisElapsed=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-yawStart).count();
    }



    return;
} */

bool decideDirection(){
    orientToNormal(); //Need to add this function in
    displaceYaw(90);
    float minDist1=minLaserDist;
    displaceYaw(-180);
    float minDist2=minLaserDist;
    if (minDist1>minDist2) displaceYaw(90);
    else displaceYaw(270);
    return;
}

void orientToNormal(){
    distGoal=minLaserDist+0.01; //Tolerance
    displaceYaw(-30);
    vel.angular.z=0.2;
    normDist=getNormDist();
    std::chrono::time_point<std::chrono::system_clock> startPt;
    while(getNormDist>distGoal || !timeout(4000, startPt)){
    }
    
}

bool timeout(uint64_t limit, std::chrono::time_point<std::chrono::system_clock> startPt){ //Non-blocking timer
    auto now = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now-startPt).count()<limit)return false;
    else return true;
    
}