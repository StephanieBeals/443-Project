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
float laserVals[639]={0.0};
float laserFirstDiff[638]={0.0};
int minLaserIdx;
float angle_increment;
bool objectDetect[3]={0};

uint8_t state = 0;

bool bumperPressed=false;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    bumper[msg->bumper]=msg->state;
    //Add condition for bumperPressed here
    for (int i=0; i<3; i++){
        if (bumper[i]==kobuki_msgs::BumperEvent::PRESSED){
            state=9;
            bumperPressed=true;
        }
    }
}
//Go to https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
//First entry is on robot right (-ve Y), going CCW around robot
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    angle_increment = msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);

    objectDetect[0]=false;
    objectDetect[1]=false;
    objectDetect[2]=false;
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    int laserValIndex=0;
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            if (minLaserDist>msg->ranges[laser_idx]){
                minLaserDist = msg->ranges[laser_idx];
                minLaserIdx=laser_idx;
            }
            
            laserVals[laser_idx]=msg->ranges[laser_idx];
            if(laser_idx!=0){
                laserFirstDiff[laser_idx]=laserVals[laser_idx]-laserVals[laser_idx-1];
            }
            if (laser_idx<219&& (laserVals[laser_idx]<0.7|| laserVals[laser_idx]==std::numeric_limits<float>::infinity())){
                objectDetect[0]=true;
                ROS_INFO("Object on right");
            } else if (laser_idx>429 &&  (laserVals[laser_idx]<0.7|| laserVals[laser_idx]==std::numeric_limits<float>::infinity())){
                objectDetect[2]=true;
                ROS_INFO("Object on left");
            } else if (laser_idx < 429 && laser_idx>219 && (laserVals[laser_idx]<0.7|| laserVals[laser_idx]==std::numeric_limits<float>::infinity())){
                objectDetect[1]=true;
                ROS_INFO("Object in front");
            }
            
            laserValIndex++;
            //if(laserValIndex>300) ROS_INFO("%d", laserValIndex);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            if (minLaserDist>msg->ranges[laser_idx]){
                minLaserDist = msg->ranges[laser_idx];
                minLaserIdx=laser_idx;
            }
            laserVals[laser_idx]=msg->ranges[laser_idx];
            if(laser_idx!=0){
                laserFirstDiff[laser_idx]=laserVals[laser_idx]-laserVals[laser_idx-1];
            }
            laserValIndex++;
        }
    }

    //ROS_INFO("First entry: %f, mid entry: %f, last entry: %f", laserVals[laserIdxStart], laserVals[300], laserVals[laserIdxEnd]);

    //ROS_INFO("Min dist: %f", minLaserDist);


}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f deg", posX, posY, yaw, RAD2DEG(yaw));
}

void navLogic();
void bumperFailsafe();
bool orientToNormal();
//Should make the below two more dynamic into just one function, bool turn(float angle)
bool turn(float turnAmt, int ranIdx);
bool moveToPt();
void decideDirection(); //decides if 90 degree left or right has most free path
void wallFollow(); //once at an obstacle turns 90 degrees left and wall follows
void avoid(); //turns left or right to avoid obstacle
float idxToAng(int idx);
bool timeout(uint64_t limit, std::chrono::time_point<std::chrono::system_clock> startPt);
void angularAdd(float *summand, float angAddend);
void spinAround(); //Spin 180 degrees twice, or use a timer
bool forward(float dist, int ranIdx);


bool ranOnce[10]={0}; //Global variable for if you want something to run only once.
float dynVar[10]={0}; //Global variables for storing things, store anything you want
uint8_t dynIdx=0; //Need to start indexing this until you return to navLogic, ie if you have multiple turns in a single function
uint8_t stepNo=0;

float laserMem[2]={0.0};

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
    state=0;
    int prevState=0;

    while(minLaserDist==std::numeric_limits<float>::infinity()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        auto now =  std::chrono::system_clock::now();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();

        if (prevState!=state){
            ROS_INFO("State: %d", state);
        }

        switch(state){ //4 min for collision avoidance (random walk w/ memory), 4 min for corner travelling
            case 0:
            navLogic();
            for (int i=0; i<10; i++){
                ranOnce[i]=false;
                dynVar[i]=0;
            }
            stepNo=0;
            dynIdx=0;
            break;

            case 1:
            ROS_INFO("StepNo: %d", stepNo);
            decideDirection();
            break;

            case 2:
            moveToPt();
            break;


            case 9:
            //bumperFailsafe
            ROS_INFO("Bumper pressed");
            state = 0;
            break;

            default:
            navLogic();
        } 

        linear=0.0;
        angular=0.0;

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    //delete laserVals;

    return 0;
}

void navLogic(){
    float distParam=0.6;
    ROS_INFO("MinLaserDist: %f", minLaserDist);
    if (minLaserDist<distParam || minLaserDist==std::numeric_limits<float>::infinity()){
        linear=0;
        state=1;
    } else if (target_x!=0.0 && target_y!=0.0){
        state=2;
    } else {
        if (minLaserDist<1){
            linear=0.2;
        } else linear=0.4;
        angular=0.0;
        state = 0;
    }

    return;
}


void decideDirection(){
    if (stepNo==0){
        if(!orientToNormal()){
            ROS_INFO("Orienting...");
            return;
        }
        stepNo++;
    }
    else if (stepNo==1){
        if(!turn(DEG2RAD(90),dynIdx)){
            return;
        } 
        stepNo++;
        dynIdx++;
        laserMem[0] = laserVals[319];
    }
    else if (stepNo==2){
        if(!turn(M_PI,dynIdx) && stepNo==2){
            return;
        } 
        stepNo++;
        dynIdx++;
        laserMem[1] = laserVals[319];
    }
    else if (stepNo==3){
        if (laserMem[0] > laserMem[1] && laserMem[0]!=std::numeric_limits<float>::infinity()) {
            if(!turn(M_PI,dynIdx) && dynIdx==2){
                return;
            }
            state = 0;
            stepNo=0;
            return;
        } 
        else if (laserMem[1]!= std::numeric_limits<float>::infinity()) {
            state = 0;
            stepNo=0;
            return;
        } else {
            if (!turn(DEG2RAD(-45), dynIdx)){
                return;
            }
            else{
                state = 0;
                stepNo=0;
                return;
            }
        }
    }
    return;
}



bool moveToPt(){
    float displace_x=target_x-posX;
    float displace_y=target_y-posY;
    float target_yaw=atan2(displace_y, displace_x);
    float distParam=0.6;
    ROS_INFO("Target_yaw: %f", target_yaw);
    if (yaw>target_yaw + 0.01){
        angular=-0.4;
    } else if (yaw<target_yaw-0.01) {
        angular=0.4;
    } else {
        angular=0.0;
        //I want to make this conditional more robust by checkign if displace is pos or negative, but cant figure it out for now
        if (minLaserDist<distParam || minLaserDist==std::numeric_limits<float>::infinity()){
            state=0;
            return;
        }
        if ((posX>target_x+0.1 || posX<target_x-0.1) || (posY>target_y+0.1 || posY<target_y-0.1)){ 
            linear=0.2;
        } else {
            linear=0.0;
            angular=0.0;
            return true;
        }
    }
    return false;
}

void wallFollow() {
    //no break out condition right now, thinking of adding in a time limit break out
    if (laserVals[laserIdxEnd]<1) {
        linear = 0.25;
        angular = 0.0;
    }
    else {
        angular = 0.1;
        linear = 0.0;
    } state = 0;
    dynVar[0]=false;
    return;
}

bool turn(float turnAmt, int ranIdx){ //CCW is +ve, turnAmt is between -pi to pi
    if (!ranOnce[ranIdx]){
        //ROS_INFO("Init dynVar");
        ranOnce[ranIdx]=true; //You have to remember in the CALLER to reset the corresponding ranOnce
        dynVar[ranIdx]=yaw;
    }
    float goal=dynVar[ranIdx];
    angularAdd(&goal, turnAmt);
    if (yaw > goal+0.08 || yaw < goal-0.08) { //Generall the leniency should be angular*2/10
        ROS_INFO("yaw: %f, goal, %f", yaw, goal);
        if (turnAmt<0){
            angular=-0.4;
        } else {
            angular=0.4;
        }
        linear = 0.0;
        return false;
    }
    else return true;
}

bool forward(float dist, int ranIdx){
    if (!ranOnce[ranIdx]){
        ranOnce[ranIdx]=true;
        dynVar[ranIdx]=laserVals[319];
    }
    float goal=dynVar[ranIdx]-dist;
    if (laserVals[319]>goal){
        angular=0.0;
        linear=0.2;
        return false;
    }return true;
}

bool orientToNormal(){
    //ROS_INFO("minLaserIdx: %d", minLaserIdx);
    //ROS_INFO("Bow dist: %f", laserVals[319]);
    linear=0.0;
    if(minLaserIdx>330){
        angular=0.1;
        return false;
    } else if (minLaserIdx<310){
        angular=-0.1;
        return false;
    } 

    return true;
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
    idx=idx-319;
    return idx*angle_increment;
}

void angularAdd(float *summand, float angAddend){
    *summand=*summand+angAddend;
    while(*summand>M_PI){
        *summand=*summand-2*M_PI;
    }
    while(*summand<0-M_PI){
        *summand=*summand+2*M_PI;
    }
    return;
}

void bumperFailSafe() {
    // Check if any of the bumpers were pressed.
    bool left_bumper_pressed = false;
    bool center_bumper_pressed = false;
    bool right_bumper_pressed = false;
    bool bumper_fail_safe_complete = false; // Fixed variable name

    // Iterate over bumpers and update pressed status
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        left_bumper_pressed |= (bumper[0] == kobuki_msgs::BumperEvent::PRESSED);
        center_bumper_pressed |= (bumper[1] == kobuki_msgs::BumperEvent::PRESSED);
        right_bumper_pressed |= (bumper[2] == kobuki_msgs::BumperEvent::PRESSED);
    }

    // Control logic after bumpers are being pressed.
    if (left_bumper_pressed) { // Simplified condition
        // starting_yaw_bumper = yaw; // keep starting pos// this is getting over ridden every time 
        if (stepNo==0){
            // Turn away from obstacle
            if (!turn(DEG2RAD(-90.0), 1)) {
                ROS_INFO("Responding to left bumper press2"); // Corrected quotes
                return;
            }
            stepNo++;
        }

        // Move forward
        if (minLaserDist != std::numeric_limits<float>::infinity()) {
            if (stepNo==1){
                if (!forward(0.50, 2)) {
                    ROS_INFO("Responding to left bumper press2"); // Corrected quotes
                    return;
                }
            stepNo++;

            }
        }

            // Turn to look back to map object
            // Later change to move forward until mapping is complete
            if (stepNo==2){
                if (!turn(DEG2RAD(180.0), 3)) {
                    ROS_INFO("Responding to left bumper press 2.5"); // Corrected quotes
                    return;
                }
            stepNo++;

            }
            if (stepNo==3){
                if (!turn(DEG2RAD(-180.0), 4)) {
                    ROS_INFO("Responding to left bumper press 3"); // Corrected quotes
                    return;
                }

            stepNo++;
            }
           
            
        else {
            decideDirection();
            ROS_INFO("Responding to left bumper press 3");
            return;
        
        }
    bumper_fail_safe_complete = true;
    //} else if (center_bumper_pressed) {
    //    
    //    if (!turn(DEG2RAD(180.0), 1)) {
    //        ROS_INFO("Responding to center bumper press"); // Corrected quotes
    //        return;  
    //    }
//
    //    if (minLaserDist != std::numeric_limits<float>::infinity()) {
    //        if (!forward(0.50, 2)) {
    //            ROS_INFO("Responding to center bumper press2"); 
    //            return;
    //        }
//
    //        // Turn to look back 
    //        if (!turn(DEG2RAD(180.0), 3)) {
    //            ROS_INFO("Responding to center bumper press 2.5"); 
    //            return;
    //        }
//
    //        if (!turn(DEG2RAD(-180.0), 4)) {
    //            ROS_INFO("Responding to center bumper press 3"); 
    //            return;
    //        }
    //        
    //        
    //    } else {
    //        decideDirection();
    //        ROS_INFO("Responding to center bumper press 4");
    //        return;   
    //    }
    //bumper_fail_safe_complete = true;
    //} else if (right_bumper_pressed) {
    //    // Turn away from obstacle
    //    if (!turn(DEG2RAD(90.0), 1)) {
    //        ROS_INFO("Responding to right bumper press2"); // Corrected quotes
    //        return;
    //    }
//
    //    // Move forward
    //    if (minLaserDist != std::numeric_limits<float>::infinity()) {
    //        if (!forward(0.50, 2)) {
    //            ROS_INFO("Responding to right bumper press2"); // Corrected quotes
    //            return;
    //        }
//
    //        // Turn to look back 
    //        if (!turn(DEG2RAD(180.0), 3)) {
    //            ROS_INFO("Responding to right bumper press 2.5"); // Corrected quotes
    //            return;
    //        }
//
    //        if (!turn(DEG2RAD(-180.0), 4)) {
    //            ROS_INFO("Responding to right bumper press 3"); // Corrected quotes
    //            return;
    //        }
    //        
    //    } else {
    //        decideDirection();
    //        ROS_INFO("Responding to right bumper press 4");
    //        return;   
    //    }
    //bumper_fail_safe_complete = true;
    //}
    
    // Update state based on bumper safety
    if (bumper_fail_safe_complete) {
        state = 0;
    } else {
        state = 9;
    }
}