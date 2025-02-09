#include "laser.h"
#include <ros/ros.h>

extern float full_angle; // If full_angle is used here, it should be declared as external in the main or another file.

void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist) {
    float angle = (float) ind / (float) nLasers * full_angle + 90 - full_angle / 2;
    horz_dist = distance * std::cos(DEG2RAD(angle));
    front_dist = distance * std::sin(DEG2RAD(angle));
    front_dist += 0.05;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    static int nLasers = static_cast<int>((msg->angle_max - msg->angle_min) / msg->angle_increment);
    static int right_idx = 0, front_idx = nLasers / 2, left_idx = nLasers - 1;
    static LaserScanData laser_data;

    laser_data.left_distance = msg->ranges[left_idx];
    laser_data.front_distance = msg->ranges[front_idx];
    laser_data.right_distance = msg->ranges[right_idx];

    while(std::isnan(laser_data.left_distance) && left_idx > 0) {
        laser_data.left_distance = msg->ranges[--left_idx];
    }

    while(std::isnan(laser_data.right_distance) && right_idx < nLasers - 1) {
        laser_data.right_distance = msg->ranges[++right_idx];
    }

    int i = 1;
    bool odd = true;
    while(std::isnan(laser_data.front_distance) && front_idx + i < nLasers && front_idx - i >= 0) {
        laser_data.front_distance = msg->ranges[front_idx + i];
        if (odd) {
            i = -i - 1;
        } else {
            i = -i;
        }
        odd = !odd;
    }

    ROS_INFO("Left: %.2f m, Front: %.2f m, Right: %.2f m, Min: %.2f m",
             laser_data.left_distance, laser_data.front_distance,
             laser_data.right_distance, laser_data.min_distance);
}
