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

//Used in laser callback
bool dynMem=false;
float* laserVals;
int minLaserIdx;

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
    if (!dynMem){
        laserVals= new float(desiredNLasers+1); //+1 just in case?
        dynMem=true;
    }


    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    int laserValIndex=0;
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            if (minLaserDist>msg->ranges[laser_idx]){
                minLaserDist = msg->ranges[laser_idx];
                minLaserIdx=laser_idx;
            }
            laserVals[laserValIndex]=msg->ranges[laser_idx];
            laserValIndex++;
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            //minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            if (minLaserDist>msg->ranges[laser_idx]){
                minLaserDist = msg->ranges[laser_idx];
                minLaserIdx=laser_idx;
            }
            laserVals[laserValIndex]=msg->[laser_idx];
            laserValIndex++;
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
void navLogic();
void bumperFailsafe();
void orientToNormal();
float idxToAng(int idx);
bool timeout(uint64_t limit, std::chrono::time_point<std::chrono::system_clock> startPt);

bool firstRun=true; //Global variable for if you want something to run only once

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
    uint8_t state=0;

    int state=0;
    float distParam=0.8;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        auto now =  std::chrono::system_clock::now();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();
        ROS_INFO("Spin start: %lu", millis);

        switch(state){
            case 0:
            navLogic();
            break;

            case 1:
            orientToNormal();
            break;

        }

        

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

        now =  std::chrono::system_clock::now();
        millis = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();
        ROS_INFO("Spin end: %lu", millis);

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep(); //Why is this here?
    }
    delete laserVals;

    return 0;
}
//navLogic, lookAround, orientToNormal, decideDirection, moveToPt, navPastObject, bumperFailsafe
//Corner detection
//Left clear, right clear as supporting funcs
void navLogic(){
    //navLogic decides which states to enter, reactive states are the first few if statements in order of priority, proactive later
    //Last state is default,move in a straight line forward
    float distParam=0.8
    if (bumperPressed){ 
        linear=0;
        angular=0;
        state=1;
    } else if (minLaserDist<distParam || minLaserDist==std::numeric_limits<float>::infinity()){
        linear=0;
        state=2;
    } else if (doLook){
        state=9;
    } else {
        linear=0.4;
        angular=0.0;
        state=0;
    }
    return;
}


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
    
    
    /*
    distGoal=minLaserDist+0.01; //Tolerance
    displaceYaw(-30);
    vel.angular.z=0.2;
    normDist=getNormDist();
    std::chrono::time_point<std::chrono::system_clock> startPt;
    while(getNormDist>distGoal || !timeout(4000, startPt)){
    }
    */
    getNormalDirection();
    angular=+/-angularVel;
    if(facingNormal){
        state=0;
    } else {return;}
    
}

bool timeout(uint64_t limit, std::chrono::time_point<std::chrono::system_clock> startPt){ //Non-blocking timer
    //Create a time point at when your timer starts and pass it in (see 'now' or 'start' for examples)
    //Returns true if the limit has passed since the timer start was initialized
    //Runs in milliseconds
    auto now = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now-startPt).count()<limit)return false;
    else return true;
}

float idxToAng(int idx){
    idx=idx-desiredNLasers/2;
    return idx*angle_increment;
}

/*
void displaceYaw(int yawDispDeg){  //Deprecated, do not use
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