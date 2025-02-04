#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <unordered_map>
#include <cmath>
#include <limits>
#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// Struct to hold laser scan data
struct LaserScanData {
    float left_distance;
    float front_distance;
    float right_distance;
    float min_distance;
};

// Global Variables
double posX = 0., posY = 0., yaw = 0.;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
LaserScanData laser_data;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    bumper[msg->bumper] = msg->state;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int nLasers = static_cast<int>((msg->angle_max - msg->angle_min) / msg->angle_increment);
    int right_idx = 0;
    int front_idx = nLasers / 2;
    int left_idx = nLasers - 1;

    // Helper function to compute the average of three rays, handling NaNs
    auto avg_range = [&](int idx) -> float {
        float sum = 0.0;
        int count = 0;

        if (idx > 0 && !std::isnan(msg->ranges[idx - 1])) {
            sum += msg->ranges[idx - 1];
            count++;
        }
        if (!std::isnan(msg->ranges[idx])) {
            sum += msg->ranges[idx];
            count++;
        }
        if (idx < nLasers - 1 && !std::isnan(msg->ranges[idx + 1])) {
            sum += msg->ranges[idx + 1];
            count++;
        }

        return (count > 0) ? (sum / count) : std::numeric_limits<float>::quiet_NaN();
    };

    // Store processed values in the struct
    laser_data.left_distance = avg_range(left_idx);
    laser_data.front_distance = avg_range(front_idx);
    laser_data.right_distance = avg_range(right_idx);

    // Compute min distance while ignoring NaNs
    laser_data.min_distance = std::numeric_limits<float>::infinity();
    for (int i = 0; i < nLasers; ++i) {
        if (!std::isnan(msg->ranges[i])) {
            laser_data.min_distance = std::min(laser_data.min_distance, msg->ranges[i]);
        }
    }

    ROS_INFO("Left: %.2f m, Front: %.2f m, Right: %.2f m, Min: %.2f m",
             laser_data.left_distance, laser_data.front_distance,
             laser_data.right_distance, laser_data.min_distance);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    const float target_distance = 0.9;
    const double k = 0.15;   // Scaling factor for angular velocity
    const double alpha = 1.5; // Exponential growth/decay rate

    while (ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        // Ensure all distances are valid before proceeding
        if (!std::isnan(laser_data.front_distance) && 
            !std::isnan(laser_data.left_distance) && 
            !std::isnan(laser_data.right_distance)) 
        {
            if (laser_data.front_distance > 1.0) {
                if (laser_data.left_distance < target_distance) {
                    vel.angular.z = -k * (1 - exp(-alpha * laser_data.left_distance)); // Exponential decay for left turns
                    vel.linear.x = 0.1;  
                } 
                else if (laser_data.left_distance > target_distance) {
                    vel.angular.z = k * (1 - exp(-alpha * laser_data.left_distance));  // Exponential decay for right turns
                    vel.linear.x = 0.1;  
                } 
                else {
                    vel.angular.z = 0.0;  
                    vel.linear.x = 0.1;  
                }
            } 
            else {
                vel.linear.x = 0.04;
                vel.angular.z = -0.26;  // Rotate in place to adjust to the right
            }
        } 
        else {
            // If sensor data is unreliable, stop movement
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            ROS_WARN("Laser scan data contains NaNs, stopping movement.");
        }

        vel_pub.publish(vel);

        // Update the timer
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - start
        ).count();

        loop_rate.sleep();
    }

    return 0;
}
