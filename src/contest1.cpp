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

float navigationTolerance = 0.2;
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
float leftDistance = 0.0, rightDistance = 0.0, centerDistance = 0.0, minDistance = 0.0;
uint16_t nLasers=0, desiredNLasers=0, desiredAngle=5;

#pragma endregion

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper] = msg->state;
    leftBumperPressed = bumper[kobuki_msgs::BumperEvent::LEFT];
    centerBumperPressed = bumper[kobuki_msgs::BumperEvent::CENTER];
    rightBumperPressed = bumper[kobuki_msgs::BumperEvent::RIGHT];
    ROS_INFO("BUMPER STATES L/C/R: %u/%u/%u", leftBumperPressed, centerBumperPressed, rightBumperPressed);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;

    // Get the indices for first, middle, and last readings
    uint16_t  right_idx = 0;                 // First reading (left)
    uint16_t  front_idx = nLasers / 2;      // Middle reading (front)
    uint16_t  left_idx = nLasers - 1;      // Last reading (right)

    leftDistance = msg->ranges[left_idx];
    centerDistance = msg->ranges[front_idx];
    rightDistance = msg->ranges[right_idx];

    minDistance = msg->ranges[0];
    for (int i=0; i< nLasers; i++){
        // msg->ranges[i] is non-NaN, update minDistance
        if(msg->ranges[i] == msg->ranges[i]){
            minDistance = std::min(minDistance, msg->ranges[i]);
        }

        // msg->ranges[i] is NaN, set minDistance to 0
        // else{
        //     minDistance = 0;
        //     break;
        // }
    }


    // ROS_INFO(msg->ranges);

    // for(int i = 0; i < nLasers; i++){
    //     ROS_INFO("dist%u : %.2f", i, msg->ranges[i]);
    // }

    // ROS_INFO("L/C/R/M Distances %.2f/%.2f/%.2f/%.2f", leftDistance, centerDistance, rightDistance, minDistance);
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

void rotateToHeading(float targetHeading, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("ROTATE TO HEADING CALLED");
    ros::spinOnce();

    float proportional;
    while(abs(targetHeading - yaw) > rotationTolerance){

        angular = computeAngular(targetHeading, yaw);

        ros::spinOnce();
        vel.angular.z = angular;
        vel.linear.x = 0;
        vel_pub.publish(vel);

        ROS_INFO("Target/Current Yaw: %f/%f degs | Setpoint: %f degs/s", targetHeading, yaw, Rad2Deg(angular));
    }

    vel.angular.z = 0;
    vel.linear.x = 0;
    vel_pub.publish(vel);

}


void navigateToPosition(float x, float y, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("navigateToPosition() CALLED");
    ros::spinOnce();
    
    float dx = x-posX;
    float dy = y-posY;
    float d = (float) sqrt(pow(dx, 2) + pow(dy, 2));

    // Set and rotate to initial heading
    float targetHeading = Rad2Deg(atan2(dy, dx));
    rotateToHeading(targetHeading, vel, vel_pub);

    // While loop until robot gets there
    while(d > navigationTolerance){
        ros::spinOnce();
        dx = x-posX;
        dy = y-posY;

        linear = computeLinear(x, y, posX, posY);

        targetHeading = Rad2Deg(atan2(dy, dx));
        angular = computeAngular(targetHeading, yaw);

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        ROS_INFO("Tgt X/Y: %f/%f | Pos X/Y: %f/%f | Lin/Ang: %f/%f", x, y, posX, posY, linear, Rad2Deg(angular));
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
    ros::spinOnce();
    bool followingWall = false;
    bool obstacleAhead = false;
    float dx, dy, d;
    float exitThreshold = 0.3;

    std::string followingMode = "none";

    //One-time major heading adjustment
    dx = tgtX-posX;
    dy = tgtY-posY;

    float targetHeading = Rad2Deg(atan2(dy, dx));
    rotateToHeading(targetHeading, vel, vel_pub);

    while(true){
        ros::spinOnce();

        // Recalculate dx, dy, d, targetHeading
        dx = tgtX-posX;
        dy = tgtY-posY;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));
        targetHeading = Rad2Deg(atan2(dy, dx));


        // Exit Condition
        if(d<exitThreshold){
            return;
        }


        // 1. Check if there are obstacles ahead
        if(minDistance < 0.51){
            obstacleAhead = true;
        }
        else {
            obstacleAhead = false;
        }
        
        // 2a. If no obstacles ahead, calculate linear/angular and continue moving to target
        if(!obstacleAhead && !followingWall){


            linear = computeLinear(tgtX, tgtY, posX, posY);

            minAngular = 0;
            angular = computeAngular(targetHeading, yaw);
            minAngular = 15;
            // angular = 0;
            ROS_INFO("Target/Current Yaw: %f/%f degs | Setpoint: %f degs/s", targetHeading, yaw, Rad2Deg(angular));
        }

        // 2b. If obstacles ahead, set followingWall to true and followingMode to left or right.
        else if(obstacleAhead && !followingWall){
            followingWall = true;

            ROS_INFO("R/L Distance %.2f/%.2f", rightDistance, leftDistance);

            if(rightDistance > leftDistance){
                followingMode = "left";
                ROS_INFO("FOLLOWING WALL ON LEFT");
            }

            else{
                followingMode = "right";
                ROS_INFO("FOLLOWING WALL ON RIGHT");
            }
        }



        // 3. If supposed to be following wall, continue following wall until exit condition triggers
        if(followingWall){
            angular = 0;
            linear = 0;

            // Right distance is greater than left -> follow wall on left
            if(followingMode == "left"){
                // ROS_INFO("FOLLOWING WALL ON LEFT");
                if (centerDistance > 1.0 && !std::isnan(centerDistance) && !std::isnan(leftDistance) && !std::isnan(rightDistance)) {
    
                    const double k = 0.15;   // Scaling factor for angular velocity
                    const double alpha = 1.5; // Exponential growth/decay rate
                    float targetDistance = 0.9;

                    if (leftDistance < targetDistance) {
                        angular = -k * (1-exp(-alpha * leftDistance)); // Exponential decay for left turns
                        linear = 0.1;                                   // Set a constant forward speed
                    } 
                    else if (leftDistance > targetDistance) {
                        angular = k * (1-exp(-alpha * leftDistance));  // Exponential decay for right turns
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
            }

            // Left distance is greater than right -> follow wall on right
            else{
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
    navigateToPositionSmart(-1.929,1.346, vel, vel_pub);
    navigateToPositionSmart(-1.7069999999999999,-1.0, vel, vel_pub);
    navigateToPositionSmart(-0.8680000000000001,1.4365, vel, vel_pub);
    navigateToPositionSmart(-0.3763333333333332,-1.1406666666666667, vel, vel_pub);
    navigateToPositionSmart(0.19299999999999984,1.5270000000000001, vel, vel_pub);
    navigateToPositionSmart(0.9543333333333335,-1.2813333333333332, vel, vel_pub);
    navigateToPositionSmart(1.2539999999999998,1.6175, vel, vel_pub);
    navigateToPositionSmart(2.285,-1.422, vel, vel_pub);
    navigateToPositionSmart(2.3149999999999995,1.708, vel, vel_pub);

    // navigateToPositionSmart(0, 1.5, vel, vel_pub);

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