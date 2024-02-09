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

//defining global variables for odom, laser, and bumper callbacks
float angular = 0.0;
float linear = 0.0;
float posX=0.0, posY=0.0, yaw=0.0;
uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist=std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=20;

//float target_x = 0;
//float target_y = 0;

//global variable to start the algorithm with a 360 spin
float spinCount = 0;

//Used in laser callback
float laserVals[639]={0.0};
float laserFirstDiff[638]={0.0};
int minLaserIdx;
float angle_increment;
bool objectDetect[3]={0};

//states defined in main function used to control robot navigation logic
uint8_t state = 0;

bool bumperPressed=false;
bool interrupt_state=false;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper]=msg->state;
    
    //if a bumper is pressed the robot enters a corresponding state and function to react
    if (bumper[0]==kobuki_msgs::BumperEvent::PRESSED) {
        state = 6;
        ROS_INFO("Left Bumper Pressed");
        bumperPressed=true;
    } else if (bumper[1]==kobuki_msgs::BumperEvent::PRESSED) {
        state = 7;
        ROS_INFO("Middle Bumper Pressed");
        bumperPressed=true;
    } else if (bumper[2]==kobuki_msgs::BumperEvent::PRESSED) {
        state = 8;
        ROS_INFO("Right Bumper Pressed");
        bumperPressed=true;
    } else {
        bumperPressed=false;
    }
    
    //DELETE
    /*for (int i=0; i<3; i++){
        if (bumper[i]==kobuki_msgs::BumperEvent::PRESSED){
            state=9;
            bumperPressed=true;
        }
        else (bumperPressed=false);
    }*/
}
//Go to https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
//First entry is on robot right (-ve Y), going CCW around robot

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //laser callback stores laser scan readings into an array to use values in code

	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    angle_increment = msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);

    //laser range is split into 3 to determine if an object is on the left, right, or center
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
            if (laser_idx<294 && (laserVals[laser_idx]<0.7|| laserVals[laser_idx]==std::numeric_limits<float>::infinity())){
                objectDetect[0]=true;
                //ROS_INFO("Object on left");
            } else if (laser_idx>344 &&  (laserVals[laser_idx]<0.7|| laserVals[laser_idx]==std::numeric_limits<float>::infinity())){
                objectDetect[2]=true;
                //ROS_INFO("Object on right");
            } else if (laser_idx < 344 && laser_idx>294 && (laserVals[laser_idx]<0.7|| laserVals[laser_idx]==std::numeric_limits<float>::infinity())){
                objectDetect[1]=true;
                //ROS_INFO("Object in front");
            }
            
            laserValIndex++;
            
            //DELETE
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

void navLogic(); //overall default navigation and conditions to switch states
//void bumperFailSafe(); //DELETE
//bool orientToNormal(); //DELETE
bool turn(float turnAmt, int ranIdx); //takes in an angle amount and turns the robot
//bool moveToPt(); //DELETE
//void decideDirection(); //decides if 90 degree left or right has most free path: DELETE
//void wallFollow(); //once at an obstacle turns 90 degrees left and wall follows: DELETE
void avoid(); //turns left, right, or spins to avoid obstacles
float idxToAng(int idx); //calculates laser scan angle associated with the array indices
bool timeout(uint64_t limit, std::chrono::time_point<std::chrono::system_clock> startPt); //starts a timer to ensure robot does not get stuck in a function
void angularAdd(float *summand, float angAddend); 
void spinAround(); //spin 360 and face direction of most space
bool forward(float dist, int ranIdx); //moves robot forward
void rightBumper(); //turns left if right bumper pressed
void centerBumper(); //turns around and faces direction of most space if center bumper pressed
void leftBumper(); //turns right if left bumper pressed


bool ranOnce[10]={0}; //Global variable for if you want something to run only once.
float dynVar[10]={0}; //Global variables for storing things, store anything you want
uint8_t dynIdx=0; //Need to start indexing this until you return to navLogic, ie if you have multiple turns in a single function
uint8_t stepNo=0;

float laserMem[8]={0.0};

bool doLook=false;
int prevState=0;

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
    
    //DELETE
    //float distParam=0.8;
    state=0;

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
            if (state==0){
                //DELETE
                //posXMem[memIdx]=posX;
                //posYMem[memIdx]=posY;
                //memIdx++;
            }
        }  
        //assigning states to each function
        prevState=state;
        switch(state){ //4 min for collision avoidance (random walk w/ memory), 4 min for corner travelling
            case 0:
            navLogic();
            for (int i=0; i<10; i++){
                ranOnce[i]=false;
                dynVar[i]=0;
            }
            for (int i=0; i<8; i++){
                laserMem[i]=0;
            }
            stepNo=0;
            dynIdx=0;
            break;

            case 1:
            //ROS_INFO("StepNo: %d", stepNo);
            avoid();
            break;

            case 4:
            spinAround();
            break;

            case 6:
            leftBumper();
            break;

            case 7:
	    if(!interrupt_state){
		    for (int i=0; i<10; i++){
	                ranOnce[i]=false;
	                dynVar[i]=0;
	            }
	            for (int i=0; i<8; i++){
	                laserMem[i]=0;
            	    }
		    interrupt_state=true;
	    }
            centerBumper();
            break;

            case 8:
            rightBumper();
            break;

            //DELETE
            /*case 9:
            bumperFailSafe();
            ROS_INFO("Bumper pressed");
            //state = 0;
            break;*/
            
            default:
            navLogic();
        } 

        
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
    //navLogic decides which states to enter, reactive states are the first few if statements in order of priority, proactive later
    //Last state is default, move in a straight line forward
    //Need some way to remember last state
    float distParam=0.7;
    float farDistParam=1.0;
    //ROS_INFO("MinLaserDist: %f", minLaserDist);

    //before doing anything else do a 360 scan to face most space
    if (spinCount==0) {
        state =4;
        spinCount=1;
    } else if (minLaserDist<distParam || minLaserDist==std::numeric_limits<float>::infinity()){
        linear=0;
        state=1;
    } else if (objectDetect[1]==true){
        linear=0.10;
        angular=0.0;
        state = 0;
    } else if (minLaserDist<farDistParam && minLaserDist>distParam) {
        linear = 0.15;
        angular = 0;
    } else {
        linear=0.25;
        angular=0.0;
        state=0;
    }

    return;
}

//DELETE function but save for contest 2
/*void decideDirection(){
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
        if (laserMem[0] > laserMem[1]) {
            if(!turn(M_PI,dynIdx) && dynIdx==2){
                return;
            }
            state = 0;
            stepNo=0;
            return;
        } 
        else{
            state = 0;
            stepNo=0;
            return;
        }
    }
    return;
}*/

//DELETE but save for contest 2
/*bool moveToPt(){
    if (target_x==0.0 && target_y==0.0){
        ROS_INFO("No target given");
    }
    float displace_x=target_x-posX;
    float displace_y=target_y-posY;
    float target_yaw=atan2(displace_y, displace_x);
    ROS_INFO("Target_yaw: %f", target_yaw);
    if (yaw>target_yaw + 0.01){
        angular=-0.4;
    } else if (yaw<target_yaw-0.01) {
        angular=0.4;
    } else {
        angular=0.0;
        //I want to make this conditional more robust by checkign if displace is pos or negative, but cant figure it out for now
        if ((posX>target_x+0.1 || posX<target_x-0.1) || (posY>target_y+0.1 || posY<target_y-0.1)){ 
            linear=0.2;
        } else {
            linear=0.0;
            angular=0.0;
            return true;
        }
    }
    return false;
}*/

bool turn(float turnAmt, int ranIdx){ //CCW is +ve, turnAmt is between -pi to pi
//the turn function takes in an angle and spins the robot that angle
    if (!ranOnce[ranIdx]){
        //ROS_INFO("Init dynVar");
        ranOnce[ranIdx]=true; //You have to remember in the CALLER to reset the corresponding ranOnce
        dynVar[ranIdx]=yaw;
    }
    float goal=dynVar[ranIdx];
    angularAdd(&goal, turnAmt);
    if (yaw > goal+0.08 || yaw < goal-0.08) { //Generall the leniency should be angular*2/10
        //ROS_INFO("yaw: %f, goal, %f", yaw, goal);
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
    //the forward function moves the robot forward
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

//DELETE save for contest 2
/*bool orientToNormal(){
    //ROS_INFO("minLaserIdx: %d", minLaserIdx);
    //ROS_INFO("Bow dist: %f", laserVals[319]);
    linear=0.0;
    if(minLaserIdx>340){
        angular=0.1;
        return false;
    } else if (minLaserIdx<300){
        angular=-0.1;
        return false;
    } 

    return true;
}*/

//DELETE do not use
/*void wallFollow() {
    //no break out condition right now, thinking of adding in a time limit break out
    orientToNormal();
    if (!turn(DEG2RAD(90),0)){
        
        return; //Breaks logic prematurely if turn is not completed
    } 
    
    ROS_INFO("Starting follow...");

    if (laserVals[laserIdxEnd]<0.92) {
        linear = 0.25;
        angular = 0.0;
    }
    else {
        angular = 0.1;
        linear = 0.0;
    } state = 0;
    dynVar[0]=false;
    return;
}*/

void spinAround(){
    //the spinAround function spins the robot 360 degrees and uses the laser scan to orient itself to the index with the most space
    //returns to state 0 when done to move forward
    
    volatile int minIndex=0;
    //ROS_INFO("Step: %d", stepNo);
    if(stepNo<8 && !turn(DEG2RAD(-45),stepNo)){
        return;
    } else if (stepNo<8){
        laserMem[stepNo] = laserVals[319];
        
        //DELETE
        //laserMem[stepNo]=minLaserDist;

        if(minLaserDist == std::numeric_limits<float>::infinity()){
            laserMem[stepNo]=0;
        }
        ROS_INFO("Mem: %d, %f", stepNo, laserMem[stepNo]);
        stepNo++;
    }
    
    if (stepNo==8){
        for (int i=1; i<8; i++){
            if(laserMem[minIndex]<laserMem[i]){
                ROS_INFO("Curr max: %f, Comp: %f", laserMem[minIndex], laserMem[i]);
                minIndex=i;
            }
            
        }ROS_INFO("maxIndex: %d", minIndex);
        if(!turn(DEG2RAD(-45*(minIndex+1)),9) && minIndex!=7){
            ROS_INFO("Ang diff: %d", -45*(minIndex+1));
            return;
        } else {
            state=0;
            return;
        }
    }
    return;
}

void avoid (){
    //the avoid function turns left if an object is sensed on the right, turns right if an object is on the left, moves forward if in a corridor,
    //and calls the spinAround function if there's objects to the left right and center
    //returns state 0 to move forward
 
    if (objectDetect[0] == true && objectDetect[2] ==false){
        angular = 0.4;
        linear = 0.1;
    } else if((objectDetect[2] == true) && objectDetect[0] == false) { 
        angular = -0.4;
        linear = 0.1;
    } else if (objectDetect[0]==true && objectDetect[2]==true && objectDetect[1]==false){
        linear=0.25;
        angular=0.0;
    } else if (objectDetect[0] == true && objectDetect[1] == true && objectDetect[2] == true) {
        state =4;
    } else {
        state =0;
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

//DELETE not used split up into 3 states instead
/*void bumperFailSafe() {
    //if() {
        //linear =-0.4;
        //angular =0;
    if(!turn(DEG2RAD(180),1)){
        return;
    } else {
        state=0;
        return;
    }
}*/

void rightBumper() {
    //the rightBumper function turns the robot left by 30 degrees if the right bumper is hit
    //returns state 0 to move forward
    if(!turn(DEG2RAD(30),1)){
        return;
    } else {
        state=0;
        return;
    }
}

void leftBumper() {
    //the leftBumper function turns the robot right by 30 degrees if the left bumper is hit
    //returns 0 to move forward
    if(!turn(DEG2RAD(-30),1)){
        return;
    } else {
        state=0;
        return;
    }
}

void centerBumper() {
    //the centerBumper function spins the robot and scans the 180 degrees behind the direction of the obstacle and faces the 
    //direction withmost space
    //returns state 0 to move forward

    volatile int minIndex=1;
    //ROS_INFO("Step: %d", stepNo);

    if(stepNo==0 && !turn(DEG2RAD(-30),stepNo)){
        return;
    } else if (stepNo==0){
        ROS_INFO("Finished 45 degree turn");
        stepNo++;
    }

    if (stepNo>0 && stepNo<5 && !turn(DEG2RAD(-60),stepNo)) {
        return;
    } else if (stepNo>0 && stepNo<5){
        laserMem[stepNo] = laserVals[319];

        if(minLaserDist == std::numeric_limits<float>::infinity()){
            laserMem[stepNo]=0;
        }
        ROS_INFO("Mem: %d, %f", stepNo, laserMem[stepNo]);
        stepNo++;
    }

    if (stepNo==5){
        for (int i=2; i<5; i++){
            if(laserMem[minIndex]<laserMem[i]){
                ROS_INFO("current max: %f, compared: %f", laserMem[minIndex], laserMem[i]);
                minIndex=i;
            }
        }ROS_INFO("max index: %d", minIndex);
        if(!turn(DEG2RAD(60*(4-minIndex)),6) && minIndex!=5){
            return;
        } else {
            state=0;
	    interrupt_state=false;
            return;
        }
    }
    return;

}
