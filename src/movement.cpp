#include "movement.h"

float rotationTolerance = Deg2Rad(5);
float kp_r = 2;
float kn_r = 0.7;
float minAngular = 15; // Degrees per second
float maxAngular = 45; // Degrees per second

float navigationTolerance = 0.1;
float navigationBumperExitTolerance = 0.5;
float kp_n = 0.02;
float kn_n = 0.5;
float minLinear = 0.1;
float maxLinear = 0.6;

float linear;
float angular;

float posX, posY, yaw;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = Rad2Deg(tf::getYaw(msg->pose.pose.orientation));
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, Rad2Deg(yaw));
}

float computeAngular(float targetHeading, float currentYaw){
    float angularDeg;

    // Calculate proportional component and then calculate angularDeg based on if it is negative or positive
    float proportional = (targetHeading-currentYaw);

    while(proportional > 180){
        proportional -= 360;
    }

    while(proportional < -180){
        proportional += 360;
    }


    if(proportional < 0){
        angularDeg = (float) -1*pow(-1*kp_r*proportional, kn_r);
    }
    else{
        angularDeg = (float) pow(kp_r*proportional, kn_r);
    }

    applyMagnitudeLimits(angularDeg, minAngular, maxAngular);

    return Deg2Rad(angularDeg);
}

float computeLinear(float tgtX, float tgtY, float posX, float posY){
    float dx = tgtX-posX;
    float dy = tgtY-posY;
    float d = (float) sqrt(pow(dx, 2) + pow(dy, 2));
    // float localLinear = (float) pow(kp_n*d, kn_n);
    float localLinear = (float) pow(kp_n*std::max(distances.min, (float) 0.1), kn_n);

    applyMagnitudeLimits(localLinear, minLinear, maxLinear);

    return localLinear;
}

void computeAdvanceCoordinate(float distance, float yaw, float posX, float posY, float &targetX, float &targetY){
    float dx = distance * std::cos(Deg2Rad(yaw));
    float dy = distance * std::sin(Deg2Rad(yaw));

    targetX = posX + dx;
    targetY = posY + dy;
}

void computeTargetCoordinate(float distance, float angle, float posX, float posY, float yaw, float &tgtX, float &tgtY){
    tgtX = posX + distance * std::cos(Deg2Rad(angle + yaw));
    tgtY = posY + distance * std::sin(Deg2Rad(angle + yaw));
}

void rotateToHeading(float targetHeading, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("rotateToHeading() called with current/target headings of %.2f/%.2f...", yaw, targetHeading);
    ros::spinOnce();
    
    float proportional;
    int counter = 0;
    
    while(targetHeading < -180){
        targetHeading += 360;
    }

    while (targetHeading > 180){
        targetHeading -= 360;
    }

    while(abs(targetHeading - yaw) > rotationTolerance){

        angular = computeAngular(targetHeading, yaw);
        linear = 0;

        ros::spinOnce();
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        counter ++;
        // if(counter %10000000 == 0){
        //     ROS_INFO("Target/Current Yaw: %f/%f degs | Setpoint: %f degs/s", targetHeading, yaw, Rad2Deg(angular));
        // }
        
    }

    vel.angular.z = 0;
    vel.linear.x = 0;
    vel_pub.publish(vel);

    ROS_INFO("...rotateToHeading() completed.");

}

void navigateToPosition(float tgtX, float tgtY, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("navigateToPosition() called with target(%.2f, %.2f)...", tgtX, tgtY);
    ros::spinOnce();

    int counter = 0;
    int bumperHits = 0;
    int bumperHitsLimit = 3;
    
    float dx = tgtX-posX;
    float dy = tgtY-posY;
    float d = (float) sqrt(pow(dx, 2) + pow(dy, 2));

    // Set and rotate to initial heading
    float targetHeading = Rad2Deg(atan2(dy, dx));
    rotateToHeading(targetHeading, vel, vel_pub);

    // While loop until robot gets there
    while(d > navigationTolerance){
        ros::spinOnce();
        dx = tgtX-posX;
        dy = tgtY-posY;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));

        if(bumpers.anyPressed && (bumperHits >= bumperHitsLimit || d < navigationBumperExitTolerance)){
            checkBumper(vel, vel_pub);
            return;
        }

        else if (bumpers.anyPressed){
            bumperHits ++;
            ROS_INFO("WARNING: BUMPER HITS: %d", bumperHits);
            checkBumper(vel, vel_pub);
            
        }
        

        linear = computeLinear(tgtX, tgtY, posX, posY);

        targetHeading = Rad2Deg(atan2(dy, dx));
        minAngular = 0;
        angular = computeAngular(targetHeading, yaw);
        minAngular = 15;

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        counter ++;
        
        // if(counter%500000 == 0){
        //     ROS_INFO("Tgt X/Y: %f/%f | Pos X/Y: %f/%f | Lin/Ang: %f/%f", tgtX, tgtY, posX, posY, linear, Rad2Deg(angular));
        // }
        
    }

    linear = 0;
    angular = 0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    ROS_INFO("...navigateToPosition completed.");
}

void navigateToPositionSmart(float tgtX, float tgtY, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("navigateToPositionSmart(...) called");
    
    // Setup
    ros::spinOnce();
    bool followingWall = false;
    bool obstacleAhead = false;
    float dx, dy, d;
    float exitThreshold = 0.45;

    float sideOpenDistThreshold = 0.7;

    float advanceDistance;
    float headingThreshold = 2; // Degrees

    std::string followingMode = "none";

    // One-time major heading adjustment to face target
    dx = tgtX-posX;
    dy = tgtY-posY;

    float targetHeading = Rad2Deg(atan2(dy, dx));
    rotateToHeading(targetHeading, vel, vel_pub);
    uint16_t counter = 0;

    // Loop
    while(true){
        ros::spinOnce();
        counter ++;
        // ROS_INFO("LEFT HORZ/RAY: %.2f/%.2f", distances.leftHorz, distances.leftRay);

        // Recalculate dx, dy, d, targetHeading
        dx = tgtX-posX;
        dy = tgtY-posY;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));
        targetHeading = Rad2Deg(atan2(dy, dx));

        checkBumper(vel, vel_pub);

        // Exit Condition
        if(d < exitThreshold){
            return;
        }


        // 1. Check if there are obstacles ahead
        if(distances.min < 0.51){
            obstacleAhead = true;
        }
        else {
            obstacleAhead = false;
        }
        
        // 2a. NO OBSTACLE MOVEMENT | If no obstacles ahead, calculate linear/angular and continue moving to target
        if(!obstacleAhead && !followingWall){


            linear = computeLinear(tgtX, tgtY, posX, posY);

            minAngular = 0;
            angular = computeAngular(targetHeading, yaw);
            minAngular = 15;

            // if(counter %50000 == 0){
            //     ROS_INFO("nTPS().2a | Tgt X/Y: %f/%f | Pos X/Y: %f/%f | Lin/Ang: %f/%f", tgtX, tgtY, posX, posY, linear, Rad2Deg(angular));
            // }
            
        }
        

        // 2b. WALL FOLLOWING SETUP | If obstacles ahead, set followingWall to true and followingMode to left or right.
        else if(obstacleAhead && !followingWall){
            followingWall = true;

            ROS_INFO("R/L Horz Distance %.2f/%.2f", distances.rightHorz, distances.leftHorz);

            if(distances.rightRay > distances.leftRay){
                followingMode = "left";
                ROS_INFO("FOLLOWING WALL ON LEFT");
            }

            else{
                followingMode = "right";
                ROS_INFO("FOLLOWING WALL ON RIGHT");
            }
        }



        // 3. WALL FOLLOWING | If supposed to be following wall, continue following wall until exit condition triggers
        if(followingWall){


            // 3A FOLLOW WALL ON LEFT | Right distance is greater than left -> follow wall on left
            if(followingMode == "left"){
                // Wall following kinematics computation
                if (distances.frontRay > 1.0) {
    
                    const double k = 0.7;   // Scaling factor for angular velocity
                    const double alpha = 1.5; // Exponential growth/decay rate
                    float targetDistance = 0.9;

                    if (distances.leftRay < targetDistance) {
                        angular = -k * (1-exp(-alpha * distances.leftRay)); // Exponential decay for left turns
                        linear = 0.1;                                   // Set a constant forward speed
                    } 
                    else if (distances.leftRay > targetDistance) {
                        angular = k * (1-exp(-alpha * distances.leftRay));  // Exponential decay for right turns
                        linear = 0.1;                                   // Set a constant forward speed
                    } 
                    else {
                        angular = 0.0;                                  // No angular adjustment
                        linear = 0.1;                                   // Maintain forward speed
                    }
                }

                else {
                    linear = 0.04;
                    angular = -0.26;                     // Rotate in place to adjust to right
                }

                // Check if sudden opening
                if(distances.leftHorzPrev < sideOpenDistThreshold && distances.leftHorz > sideOpenDistThreshold){
                    ROS_INFO("OPENING DETECTED WHILE FOLLOWING WALL ON LEFT");
                    followingMode = "leftAdvanced";
                    advanceDistance = distances.leftVertPrev + 0.1;
                    float tgtX;
                    float tgtY;

                    computeAdvanceCoordinate(advanceDistance, yaw, posX, posY, tgtX, tgtY);
                    navigateToPosition(tgtX, tgtY, vel, vel_pub);
                }


            }

            else if (followingMode == "leftAdvanced"){
                linear = 0.1;
                angular = Deg2Rad((float) 15);
                
                // Return to normal navigation if turned to the correct heading
                if(std::abs(targetHeading - yaw) < headingThreshold && distances.frontRay > 0.51){
                    followingMode = "none";
                    followingWall = false;

                }
                
            }

            // 3B FOLLOW WALL ON RIGHT | Left distance is greater than right -> follow wall on right
            else if (followingMode == "right"){
                // Wall following kinematics computation
                if (distances.frontRay > 1.0) {
                    const double k = 0.7;   // Scaling factor for angular velocity
                    const double alpha = 1.5; // Exponential growth/decay rate
                    float targetDistance = 0.9;

                    if (distances.rightRay < targetDistance) {
                        angular = k * (1-exp(-alpha * distances.rightRay)); // Exponential decay for left turns
                        linear = 0.1;                                   // Set a constant forward speed
                    } 
                    else if (distances.rightRay > targetDistance) {
                        angular = -k * (1-exp(-alpha * distances.rightRay));  // Exponential decay for right turns
                        linear = 0.1;                                   // Set a constant forward speed
                    } 
                    else {
                        angular = 0.0;                                  // No angular adjustment
                        linear = 0.1;                                   // Maintain forward speed
                    }
                }

                else {
                    linear = 0.04;
                    angular = 0.26;                     // Rotate in place to adjust to right
                }

                // Check if sudden opening
                if(distances.rightHorzPrev < sideOpenDistThreshold && distances.rightHorz > sideOpenDistThreshold){
                    ROS_INFO("OPENING DETECTED WHILE FOLLOWING WALL ON RIGHT");
                    followingMode = "rightAdvanced";
                    advanceDistance = distances.rightVertPrev + 0.1;
                    float tgtX;
                    float tgtY;

                    computeAdvanceCoordinate(advanceDistance, yaw, posX, posY, tgtX, tgtY);
                    navigateToPosition(tgtX, tgtY, vel, vel_pub);
                }
                //ROS_INFO("FOLLOWING WALL ON RIGHT");
            }
        

            else if (followingMode == "rightAdvanced"){
                linear = 0.1;
                angular = Deg2Rad((float) -15);
                
                // Return to normal navigation if turned to the correct heading
                if(std::abs(targetHeading - yaw) < headingThreshold && distances.frontRay > 0.51){
                    followingMode = "none";
                    followingWall = false;

                }
                
            }

        }
        // else {
        //     linear = 0.0;
        //     angular = 0.0;
        //     ROS_INFO("navigateToPositionSmart: CASE NOT HANDLED");
        // }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }
    
    angular = 0;
    linear = 0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
}
