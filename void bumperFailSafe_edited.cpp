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
        
        // Turn away from obstacle
        if (!turn(DEG2RAD(-90.0), 1)) {
            ROS_INFO("Responding to left bumper press2"); // Corrected quotes
            return;
        }

        // Move forward
        if (minLaserDist != std::numeric_limits<float>::infinity()) {
            if (!forward(0.50, 2)) {
                ROS_INFO("Responding to left bumper press2"); // Corrected quotes
                return;
            }

            // Turn to look back to map object
            // Later change to move forward until mapping is complete
            if (!turn(DEG2RAD(180.0), 3)) {
                ROS_INFO("Responding to left bumper press 2.5"); // Corrected quotes
                return;
            }

            if (!turn(DEG2RAD(-180.0), 4)) {
                ROS_INFO("Responding to left bumper press 3"); // Corrected quotes
                return;
            }
            
            
        } else {
            decideDirection();
            ROS_INFO("Responding to left bumper press 3");
            return;
        
        }
    bumper_fail_safe_complete = true;
    } else if (center_bumper_pressed) {
        if (!turn(DEG2RAD(180.0), 1)) {
            ROS_INFO("Responding to center bumper press"); // Corrected quotes
            return;  
        }

        if (minLaserDist != std::numeric_limits<float>::infinity()) {
            if (!forward(0.50, 2)) {
                ROS_INFO("Responding to center bumper press2"); 
                return;
            }

            // Turn to look back 
            if (!turn(DEG2RAD(180.0), 3)) {
                ROS_INFO("Responding to center bumper press 2.5"); 
                return;
            }

            if (!turn(DEG2RAD(-180.0), 4)) {
                ROS_INFO("Responding to center bumper press 3"); 
                return;
            }
            
            
        } else {
            decideDirection();
            ROS_INFO("Responding to center bumper press 4");
            return;   
        }
    bumper_fail_safe_complete = true;
    } else if (right_bumper_pressed) {
        // Turn away from obstacle
        if (!turn(DEG2RAD(90.0), 1)) {
            ROS_INFO("Responding to right bumper press2"); // Corrected quotes
            return;
        }

        // Move forward
        if (minLaserDist != std::numeric_limits<float>::infinity()) {
            if (!forward(0.50, 2)) {
                ROS_INFO("Responding to right bumper press2"); // Corrected quotes
                return;
            }

            // Turn to look back 
            if (!turn(DEG2RAD(180.0), 3)) {
                ROS_INFO("Responding to right bumper press 2.5"); // Corrected quotes
                return;
            }

            if (!turn(DEG2RAD(-180.0), 4)) {
                ROS_INFO("Responding to right bumper press 3"); // Corrected quotes
                return;
            }
            
        } else {
            decideDirection();
            ROS_INFO("Responding to right bumper press 4");
            return;   
        }
    bumper_fail_safe_complete = true;
    }
    
    // Update state based on bumper safety
    if (bumper_fail_safe_complete) {
        state = 0;
    } else {
        state = 9;
    }
}
