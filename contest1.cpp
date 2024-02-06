bool isCorner(const sensor_msgs::LaserScan::ConstPtr& msg){
    std::vector<float> differences; //first difference array 

    for (size_t i = 1; i < msg->ranges.size(); ++i){
        float diff = std::abs(msg->ranges[i] - msg->ranges[i-1]);
        //ROS_INFO("msg->ranges[i]: %i", msg->ranges[i]);
        //ROS_INFO("msg->ranges[i-1]: %i", msg->ranges[i-1]);
        //ROS_INFO("difference: %i", diff);
        differences.push_back(diff);
    }
    
    int consecutive_count = 0;
    size_t corner_index = 0;
    float threshold = 0.005;
    int consecutive_threshold = 400;

    for (size_t i = 1; i < differences.size(); ++i){
        //ROS_INFO("difference: %i", differences[i]);
    }

    for (size_t i = 1; i < differences.size(); ++i){
        if (differences[i] < threshold){
            consecutive_count++;
            if (consecutive_count == 1) {
                corner_index = i;
            } 
            if (consecutive_count >= consecutive_threshold){
                return true;
                }
            }
            else{
                consecutive_count = 0;
            }
        }
    return false;
    }

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    bool cornerDetected = isCorner(msg);

    if (cornerDetected) {
        ROS_INFO("CORNER!");
    }
    else {
        ROS_INFO("no corner");
    }

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
}
