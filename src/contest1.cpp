//PERCY
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

#include <math.h>
#include <string>

#define N_BUMPER (3)
#define Rad2Deg(rad) ((rad) * 180. / M_PI)
#define Deg2Rad(deg) ((deg) * M_PI / 180.)

#pragma region Kinematics
float angular;
float linear;
float posX = 0.0, posY = 0.0, yaw = 0.0;

float rotationTolerance = Deg2Rad(1);
float kp_r = 1;
float kn_r = 0.7;
float minAngular = 15; // Degrees per second
float maxAngular = 90; // Degrees per second

float navigationTolerance = 0.05;
float kp_n = 0.02;
float kn_n = 0.5;
float minLinear = 0.1;
float maxLinear = 0.45;
#pragma endregion

#pragma region Bumper
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool leftBumperPressed;
bool centerBumperPressed;
bool rightBumperPressed;
#pragma endregion

#pragma region Laser
struct DistancesStruct{
    float leftRay;
    float leftRayPrev;
    float leftHorz;
    float leftHorzPrev;
    float leftVert;
    float leftVertPrev;

    float frontRay;
    float frontRayPrev;

    float rightRay;
    float rightRayPrev;
    float rightHorz;
    float rightHorzPrev;
    float rightVert;
    float rightVertPrev;

    float min;
    float minPrev;
};

DistancesStruct distances;

uint16_t nLasers=0;
float fullAngle = 57.0;

#pragma endregion

#pragma region Laser Obstacle Avoidance


#pragma endregion


void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist){
    float angle = (float) ind / (float) nLasers * fullAngle + 90 - fullAngle/2;
    horz_dist = std::abs(distance * std::cos(Deg2Rad(angle)));
    front_dist = std::abs(distance * std::sin(Deg2Rad(angle)));
}

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper] = msg->state;
    leftBumperPressed = bumper[kobuki_msgs::BumperEvent::LEFT];
    centerBumperPressed = bumper[kobuki_msgs::BumperEvent::CENTER];
    rightBumperPressed = bumper[kobuki_msgs::BumperEvent::RIGHT];
    ROS_INFO("BUMPER STATES L/C/R: %u/%u/%u", leftBumperPressed, centerBumperPressed, rightBumperPressed);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // 1. Update previous values
    distances.leftRayPrev = distances.leftRay;
    distances.leftHorzPrev = distances.leftHorz;
    distances.leftVertPrev = distances.leftVert;

    distances.frontRayPrev = distances.frontRay;

    distances.rightRayPrev = distances.rightRay;
    distances.rightHorzPrev = distances.rightHorz;
    distances.rightVertPrev = distances.rightVert;

    distances.minPrev = distances.min;

    // ROS_INFO("CURR %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", distances.leftRay, distances.leftHorz, distances.leftVert, distances.frontRay, distances.rightRay, distances.rightHorz, distances.rightVert, distances.min);
    // ROS_INFO("PREV %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", distances.leftRayPrev, distances.leftHorzPrev, distances.leftVertPrev, distances.frontRayPrev, distances.rightRayPrev, distances.rightHorzPrev, distances.rightVertPrev, distances.minPrev);


    // 2. Get the indices for first, middle, and last readings
    uint16_t  rightInd = 0;               // First reading (right)
    uint16_t  frontInd = nLasers / 2;     // Middle reading (front)
    uint16_t  leftInd = nLasers - 1;      // Last reading (left)

    // 3. Find the closes non-nan value for right, front, and left
    distances.leftRay= msg->ranges[leftInd];
    distances.frontRay= msg->ranges[frontInd];
    distances.rightRay = msg->ranges[rightInd];

    // 3a Left
    while(std::isnan(distances.leftRay)){
        distances.leftRay = msg->ranges[leftInd];
        leftInd--;

        if(leftInd < frontInd){
            distances.leftRay = 0.25;
            break;
        }
    }

    // 3b Right
    while(std::isnan(distances.rightRay)){
        distances.rightRay = msg->ranges[rightInd];
        rightInd++;

        if(rightInd > frontInd){
            distances.rightRay=0.25;
            break;
        }
    }

    // 3c Front
    int i = 1;
    bool odd = true;

    while(std::isnan(distances.frontRay)){
        distances.frontRay = msg->ranges[frontInd+i];
        if(odd){
            odd = false;
            i = -(i);
        }
        else{
            odd = true;
            i = -(i+1);
        }
    }

    frontInd += i;

    // 4. Calculate Orthogonal (Horz/Vert) for Left and Right
    // 4a Left
    orthogonalizeRay(leftInd, nLasers, distances.leftRay, distances.leftHorz, distances.leftVert);
    
    // 4b Right
    orthogonalizeRay(rightInd, nLasers, distances.rightRay, distances.rightHorz, distances.rightVert);

    // 5. Calculate min Distance
    distances.min = std::min(std::min(distances.rightRay, distances.frontRay), distances.leftRay);


}


void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg){
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = Rad2Deg(tf::getYaw(msg->pose.pose.orientation));
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, Rad2Deg(yaw));
}

float absPow(float base, float exp){
    if(base < 0){
        return (float) -1*pow(-1*base, exp);
    }
    else{
        return (float) pow(base, exp);
    } 
}

void applyMagnitudeLimits(float &value, float lowerLimit, float upperLimit){
    if(value < 0){
        if(value < -upperLimit){
            value = -upperLimit;
        }
        else if(value > -lowerLimit){
            value = -lowerLimit;
        }
    }

    else if(value > 0){
        if(value > upperLimit){
            value = upperLimit;
        }
        else if(value < lowerLimit){
            value = lowerLimit;
        }
    }
}

float computeAngular(float targetHeading, float currentYaw){
    float angularDeg;

    // Calculate proportional component and then calculate angularDeg based on if it is negative or positive
    float proportional = kp_r*(targetHeading-currentYaw);

    if(proportional > 180){
        proportional -= 360;
    }

    else if(proportional < -180){
        proportional += 360;
    }


    if(proportional < 0){
        angularDeg = (float) -1*pow(-1*proportional, kn_r);
    }
    else{
        angularDeg = (float) pow(proportional, kn_r);
    }

    applyMagnitudeLimits(angularDeg, minAngular, maxAngular);

    return Deg2Rad(angularDeg);
}

float computeLinear(float tgtX, float tgtY, float posX, float posY){
    float dx = tgtX-posX;
    float dy = tgtY-posY;
    float d = (float) sqrt(pow(dx, 2) + pow(dy, 2));
    float localLinear = (float) pow(kp_n*d, kn_n);
    applyMagnitudeLimits(localLinear, minLinear, maxLinear);

    return localLinear;
}

void computeAdvanceCoordinate(float distance, float yaw, float posX, float posY, float &targetX, float &targetY){
    float dx = distance * std::cos(Deg2Rad(yaw));
    float dy = distance * std::sin(Deg2Rad(yaw));

    targetX = posX + dx;
    targetY = posY + dy;
}

void rotateToHeading(float targetHeading, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("ROTATE TO HEADING CALLED");
    ros::spinOnce();

    float proportional;
    while(abs(targetHeading - yaw) > rotationTolerance){

        angular = computeAngular(targetHeading, yaw);
        linear = 0;

        ros::spinOnce();
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        ROS_INFO("Target/Current Yaw: %f/%f degs | Setpoint: %f degs/s", targetHeading, yaw, Rad2Deg(angular));
    }

    vel.angular.z = 0;
    vel.linear.x = 0;
    vel_pub.publish(vel);

}


void navigateToPosition(float tgtX, float tgtY, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("navigateToPosition() CALLED");
    ros::spinOnce();

    int counter = 0;
    
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

        linear = computeLinear(tgtX, tgtY, posX, posY);

        targetHeading = Rad2Deg(atan2(dy, dx));
        minAngular = 0;
        angular = computeAngular(targetHeading, yaw);
        minAngular = 15;

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        counter ++;
        
        if(counter%500000 == 0){
            ROS_INFO("Tgt X/Y: %f/%f | Pos X/Y: %f/%f | Lin/Ang: %f/%f", tgtX, tgtY, posX, posY, linear, Rad2Deg(angular));
        }
        
    }

    linear = 0;
    angular = 0;
    vel.angular.z = angular;
    vel.linear.x = 0;
    vel_pub.publish(vel);

    ROS_INFO("Successful exit from navigateToPosition.");
}

void navigateToPositionSmart(float tgtX, float tgtY, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("navigateToPositionSmart(...) called");
    
    // Setup
    ros::spinOnce();
    bool followingWall = false;
    bool obstacleAhead = false;
    float dx, dy, d;
    float exitThreshold = 0.3;

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

            if(counter %50000 == 0){
                ROS_INFO("nTPS().2a | Tgt X/Y: %f/%f | Pos X/Y: %f/%f | Lin/Ang: %f/%f", tgtX, tgtY, posX, posY, linear, Rad2Deg(angular));
            }
            
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
    
                    const double k = 0.15;   // Scaling factor for angular velocity
                    const double alpha = 2.3; // Exponential growth/decay rate
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
                angular = Deg2Rad((float) 11.4);
                
                // Return to normal navigation if turned to the correct heading
                if(std::abs(targetHeading - yaw) < headingThreshold && distances.frontRay > 0.51){
                    followingMode = "none";
                    followingWall = false;

                }
                
            }

            // 3B FOLLOW WALL ON RIGHT | Left distance is greater than right -> follow wall on right
            else if (followingMode == "right"){
                //ROS_INFO("FOLLOWING WALL ON RIGHT");
            }
        }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber subOdom = nh.subscribe("odom", 1, &callbackOdom);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Bumper Event Settings
    bool bumperStepBack = false;
    uint64_t tBumperEventStart;
    uint64_t dBumperStepBack = 2;

    uint64_t dBumperEvent[4] = {3,7,3,7};
    uint64_t dBumperEventTotal = 0;
    for(int i = 0; i < sizeof(dBumperEvent)/sizeof(*dBumperEvent); i++){
        dBumperEventTotal += dBumperEvent[i];
    }

    angular = 0.0;
    linear = 0.0;

    // rotateToHeading(90, vel, vel_pub);
    // rotateEndlessly(vel, vel_pub);
    // navigateToPositionSmart(-1.929,1.346, vel, vel_pub);
    // navigateToPositionSmart(-1.7069999999999999,-1.0, vel, vel_pub);
    // navigateToPositionSmart(-0.8680000000000001,1.4365, vel, vel_pub);
    // navigateToPositionSmart(-0.3763333333333332,-1.1406666666666667, vel, vel_pub);
    // navigateToPositionSmart(0.19299999999999984,1.5270000000000001, vel, vel_pub);
    // navigateToPositionSmart(0.9543333333333335,-1.2813333333333332, vel, vel_pub);
    // navigateToPositionSmart(1.2539999999999998,1.6175, vel, vel_pub);
    // navigateToPositionSmart(2.285,-1.422, vel, vel_pub);
    // navigateToPositionSmart(2.3149999999999995,1.708, vel, vel_pub);

    navigateToPositionSmart(-2, 0, vel, vel_pub);

    return 0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        #pragma region Bumper
        // Bumper Event
        if(leftBumperPressed || centerBumperPressed || rightBumperPressed){
            bumperStepBack = true;
            tBumperEventStart = secondsElapsed;
        }

        if(bumperStepBack){
            uint64_t dBumperEventRemaining = secondsElapsed - tBumperEventStart;

            if(dBumperEventRemaining < dBumperEvent[0]){
                linear = -0.1;
                angular = 0.0;
            }
            
            else if(dBumperEventRemaining < dBumperEvent[0] + dBumperEvent[1]){
                linear = 0.0;
                angular = Deg2Rad(15);
            }

            else if(dBumperEventRemaining < dBumperEvent[0] + dBumperEvent[1] + dBumperEvent[2]){
                linear = 0.1;
                angular = 0.0;
            } 
            
            else if (dBumperEventRemaining < dBumperEvent[0] + dBumperEvent[1] + dBumperEvent[2] + dBumperEvent[3]){
                linear = 0.0;
                angular = Deg2Rad(-15);
            }

            else{
                bumperStepBack = false;
                angular = 0.0;
                linear = 0.1;
            }
        }
        #pragma endregion

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}