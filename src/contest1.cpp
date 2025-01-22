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

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

double posX = 0., posY = 0., yaw = 0.;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
float angular = 0.0;
float linear = 0.0;
float minLaserDist = std::numeric_limits<float>::infinity();
// float maxLaserDist = std::numeric_limits<float>::();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg->state;
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %frad or %fdegrees.", posX, posY, yaw, RAD2DEG(yaw));
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    minLaserDist = std::numeric_limits<float>::infinity();
    // maxLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);

    // Get the indices for first, middle, and last readings
    int left_idx = 0;                 // First reading (left)
    int front_idx = nLasers / 2;      // Middle reading (front)
    int right_idx = nLasers - 1;      // Last reading (right)
    
    float left_distance = msg->ranges[left_idx];
    float front_distance = msg->ranges[front_idx];
    float right_distance = msg->ranges[right_idx];

    // Log the results
    ROS_INFO("Left (first) distance: %.2f m", left_distance);
    ROS_INFO("Front (middle) distance: %.2f m", front_distance);
    ROS_INFO("Right (last) distance: %.2f m", right_distance);
    
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
             std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }

    ROS_INFO("Min distance: %i", minLaserDist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback); 

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        // ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, maxLaserRange);
        //fill with your code
        // Check if any of the bumpers were pressed.
        bool any_bumper_pressed = false;
        
        // Set target distance to the wall
        float target_distance = 0.5;

        // Wall following based on the laser readings
        if (left_distance < target_distance) {
            // Too close to the wall, move away
            angular = 0.2;   // Turn right
            linear = 0.1; 
        } 
        else if (left_distance > target_distance) {
            // Too far from the wall, move closer
            angular = -0.2;  // Turn left
            linear = 0.1;  
        } 
        else {
            // Maintain distance from the wall
            angular = 0.0;   // No turning
            linear = 0.1; 
        }

        '''
        // if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed && minLaserDist > 0.7) {
        //     angular = 0.0;
        //     linear = 0.2;
        // }
        // else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed && minLaserDist > 0.5) {
        //     angular = M_PI / 6;
        //     linear = 0.0;
        // }
        // else if (minLaserDist > 1. && !any_bumper_pressed) {
        //     linear = 0.1;
        // if (yaw < 17 / 36 * M_PI || posX > 0.6) {
        //     angular = M_PI / 12.;
        // }
        // else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
        //     angular = -M_PI / 12.;
        // }
        // else {
        //     angular = 0;
        // }
        // }
        // else {
        //     angular = 0.0;
        //     linear = 0.0;
        //     //break;
        // }
        '''

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
