// new headers
#include <typeinfo>
#include <iostream>
#include <utility> // for std::pair
#include <iomanip>

// cornerRecognition -> detect corner and relative valley direction
std::tuple<bool, bool, int> cornerRecognition(const sensor_msgs::LaserScan::ConstPtr& msg) {
    bool corner_detected, valley_on_left;
    int corner_laser_index;

    std::vector<float> avg_laser; //average reading over every 4 readings
    std::vector<float> laser; //laser readings
    std::vector<float> first_diff; //first difference on avg_laser
    std::vector<float> first_diff_interval; //first difference interval check when suspecting a corner
    
    for (size_t i = 0; i < msg->ranges.size(); ++i){
        float reading = msg->ranges[i];
        laser.push_back(reading);
    }

    //sanity checking for laser readings 
    for (size_t i = 0; i < laser.size(); ++i){
        //#std::cout << "index" << i << std::endl;
        //#ROS_INFO("laser: %f", laser[i]);
        //std::cout << "Type is:" << typeid(avg_laser[i]).name() << std::endl;
        //bool check = std::isnan(laser[i]); 
        //std::cout << check << std::endl;
        
        if (i == 400){
            break;
        }
        
    }

    corner_detected = false;

    for (size_t i = 0; i < laser.size(); ++i){
        if (corner_detected){
            //std::cout << "Corner Detected." << std::endl;
            break;
        } 
        bool laser_nan = std::isnan(laser[i]);
        if (laser_nan){
            continue;
        }
        float diff = std::abs(laser[i] - laser[i+1]);
        bool diff_nan = std::isnan(diff);
        int k = i;

        //skipping nan values for checking ahead from reading i
        while (diff_nan){
            k++;
            diff = std::abs(laser[i] - laser[k]);
            diff_nan = std::isnan(diff);
        }
        if (diff >= 0.5 && i <= 623 && k <= 623){ // laser array size is 649
            int false_alarm_counter = 0;
            first_diff_interval.clear();
            
            for (size_t j = i; j < k+16; ++j){
                float signed_diff_extended = laser[i] - laser[j];
                float diff_extended = std::abs(signed_diff_extended);
                bool diff_extended_nan = std::isnan(diff_extended);
                if (diff_extended_nan){
                    continue;
                }
                else if (diff_extended >= 0.5){
                    //std::cout << "large difference detected #" << j << std::endl;
                    first_diff_interval.push_back(signed_diff_extended);
                    //std::cout << "first_diff_interval_size:" << first_diff_interval.size() << std::endl;
                    //std::cout << "signed_diff_extended:" << signed_diff_extended << std::endl;
                    false_alarm_counter = 0;
                    if (j == k+15){
                        corner_detected = true;
                        corner_laser_index = i;

                        // check the sign of average first difference to determine the relative position of valley //
                        float sum = 0;
                
                        for (size_t a = 0; a < first_diff_interval.size(); ++a){
                            //signed_diff_extended : first_diff_interval
                            sum += first_diff_interval[a];
                        }
                        float average = sum / first_diff_interval.size();

                        //std::cout << "average is:" << average << std::endl;
                        if (average <= 0){
                            valley_on_left = true;
                            std::cout << "valley on the left" << std::endl;
                        } else {
                            valley_on_left = false;
                            std::cout << "valley on the right" << std::endl;
                        }
                        break;
                    } else {
                        continue;
                    }
                } 
                else{
                    false_alarm_counter++;
                    //std::cout << "false alarm:" << false_alarm_counter << std::endl;
                    if (false_alarm_counter >= 4){
                        std::cout << "FALSE ALARM!" << std::endl;
                        corner_detected = false;
                        break; 
                    }
                }
            }
        }
    }
    if (corner_detected != true){
        //std::cout << "No Corner." << std::endl;
    } 
    return std::make_tuple(corner_detected, valley_on_left, corner_laser_index);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    ////// 
    auto result = cornerRecognition(msg);
    bool result1 = std::get<0>(result); // (bool) corner detected?
    bool result2 = std::get<1>(result); // (bool) valley on the left?
    int result3 = std::get<2>(result); // (int) corner_laser_index

    if (result1){
        if (result2){
            std::cout << "yes corner: left valley detected" << std::endl;
            std::cout << "corner laser index: " << result3 << std::endl;
        } else {
            std::cout << "yes corner: right valley detected" << std::endl;
            std::cout << "corner laser index: " << result3 << std::endl;
        }
    } else {
        std::cout << "no corner" << std::endl;
    }
    //////

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
