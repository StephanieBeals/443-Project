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
#define laserIdxStart nLasers/2-desiredNLasers
#define laserIdxEnd nLasers / 2 + desiredNLasers - 1

using Clock = std::chrono::system_clock;

float angular = 0.0;
float linear = 0.0;
float posX=0.0, posY=0.0, yaw=0.0;
uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist=std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=15;

float target_x = 0;
float target_y = 0;

//Used in laser callback
bool dynMem=false;
float laserVals[639]={0.0};
int minLaserIdx;
float angle_increment;

bool bumperPressed=false;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    bumper[msg->bumper]=msg->state;
    //Add condition for bumperPressed here
}
//Go to https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
//First entry is on robot right (-ve Y), going CCW around robot
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    angle_increment = msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    if (!dynMem){
        //laserVals= new float(desiredNLasers*2+1); //+1 just in case?
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
            laserVals[laser_idx]=msg->ranges[laser_idx];
            laserValIndex++;
            //if(laserValIndex>300) ROS_INFO("%d", laserValIndex);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            //minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            if (minLaserDist>msg->ranges[laser_idx]){
                minLaserDist = msg->ranges[laser_idx];
                minLaserIdx=laser_idx;
            }
            laserVals[laser_idx]=msg->ranges[laser_idx];
            laserValIndex++;
        }
    }

    //ROS_INFO("First entry: %f, mid entry: %f, last entry: %f", laserVals[laserIdxStart], laserVals[300], laserVals[laserIdxEnd]);

    ROS_INFO("Min dist: %f", minLaserDist);


}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f deg", posX, posY, yaw, RAD2DEG(yaw));
}

void navLogic();
void bumperFailsafe();
void orientToNormal();
void turnLeft(); //turns 90 degrees left
void turnRight(); //turns 90 degrees right
void moveToPt();
void decideDirection(); //decides if 90 degree left or right has most free path
void wallFollow(); //once at an obstacle turns 90 degrees left and wall follows
void avoid(); //turns left or right to avoid obstacle
float idxToAng(int idx);
bool timeout(uint64_t limit, std::chrono::time_point<std::chrono::system_clock> startPt);
float angularAdd(float *summand, float angAddend);
void spinAround(); //Spin 180 degrees twice, or use a timer


bool firstRun=true; //Global variable for if you want something to run only once

uint8_t state=0;
bool doLook=false;

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
    

    float distParam=0.8;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        auto now =  std::chrono::system_clock::now();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();
        //ROS_INFO("Spin start: %lu", millis);

        now =  std::chrono::system_clock::now();
        millis = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();
        //ROS_INFO("Spin end: %lu", millis);


        //all following lines are for testing wall following and decision making functions
        float stopDist = 0.5;

        linear = 0.25;
        angular = 0.0;
        
        if(minLaserDist <= stopDist || minLaserDist==std::numeric_limits<float>::infinity()) {
            wallFollow();
        }


        //this is pseudocode for state 0 algorithm navLogic
        //set a minimum sensing distance at which you stop
        
        /*stopDist = 0.5;
        spinAround();

        if (check if bumpers are hit) {
            bumperFailsafe();
        }
        else if (you get within the stopping range) {
            if (it is the first two minutes of the run) {
                wallFollow();
            }
            else if (it is minutes 2 - 5) {
                avoid();
            }
            else {
                decideDirection();
            }
        }
        else {
            linear = 0.25;
            0.0;
        }*/


        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    //delete laserVals;

    return 0;
}


void turnLeft(){
    //turns left 90 degrees
    float startOrientation = yaw;
    if (yaw-startOrientation < M_PI/2) {
        angular = 0.1;
        linear = 0.0;
    }
}

void turnRight(){
    //turn right 90 degrees
    float startOrientation = yaw;
    if (startOrientation-yaw < M_PI/2) {
        angular = -0.1;
        linear = 0.0;
    }
}

void orientToNormal(){
    ROS_INFO("minLaserIdx: %d", minLaserIdx);
    ROS_INFO("Bow dist: %f", laserVals[319]); 
    linear=0.0;
    if(minLaserIdx>340){
        angular=0.1;
    } else if (minLaserIdx<300){
        angular=-0.1;
    } else state=0;

    return;
}

void decideDirection(){
    orientToNormal();
    turnLeft();
    float bowDistLeft = laserVals[319];
    turnLeft();
    turnLeft();
    float bowDistRight = laserVals[319];
    if (bowDistLeft > bowDistRight) {
        turnRight();
        turnRight();
        state = 0;
    }
    else {
        state = 0;
    }
}

void wallFollow() {
    //no break out condition right now, thinking of adding in a time limit break out
    orientToNormal();
    turnLeft();
    if (minLaserIdx>595) {
        linear = 0.25;
        angular = 0.0;
    }
    else {
        angular = 0.1;
        linear = 0.0;
    }
}

void avoid() {
    if (minLaserIdx > 319) {
        if (minLaserDist<0.5){
            angular = 0.1;
            linear = 0.0;
        } 
        state = 0;
    } else if (minLaserIdx <= 319) {
        if (minLaserDist>0.5) {
            angular = -0.1;
            linear = 0.0;
        }
        state = 0;
    } else {
        state = 0;
    }
}

bool timeout(uint64_t limit, std::chrono::time_point<std::chrono::system_clock> startPt){ //Non-blocking timer
    //Create a time point at when your timer starts and pass it in to this function (see 'now' or 'start' for examples)
    //Returns true if the limit has passed since the timer start was initialized
    //Runs in milliseconds
    auto now = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now-startPt).count()<limit)return false;
    else return true;
}

float idxToAng(int idx){ //Takes the laser index and returns the angle. Can be negative
    idx=idx-nLasers;
    return idx*angle_increment;
}

float angularAdd(float *summand, float angAddend){
    return angAddend;
}