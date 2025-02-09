#include "laser.h"
#include <ros/ros.h>

#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#define Rad2Deg(rad) ((rad) * 180. / M_PI)
#define Deg2Rad(deg) ((deg) * M_PI / 180.)

float full_angle = 57.0;
DistancesStruct distances;
int nLasers = 0;       // 激光雷达数据的数量
int right_idx = 0;     // 右侧激光索引
int front_idx = 0;     // 前方激光索引
int left_idx = 0;      // 左侧激光索引

void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist){
    float angle = (float) ind / (float) nLasers * full_angle + 90 - full_angle/2;
    horz_dist = std::abs(distance * std::cos(Deg2Rad(angle)));
    front_dist = std::abs(distance * std::sin(Deg2Rad(angle)));
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    static int nLasers = static_cast<int>((msg->angle_max - msg->angle_min) / msg->angle_increment);
    static int right_idx = 0, front_idx = nLasers / 2, left_idx = nLasers - 1;
    static LaserScanData laser_data;

    laser_data.left_distance = msg->ranges[left_idx];
    laser_data.front_distance = msg->ranges[front_idx];
    laser_data.right_distance = msg->ranges[right_idx];

    // 1. Update previous values
    distances.leftRayPrev = distances.leftRay;
    distances.leftHorzPrev = distances.leftHorz;
    distances.leftVertPrev = distances.leftVert;
    distances.frontRayPrev = distances.frontRay;
    distances.rightRayPrev = distances.rightRay;
    distances.rightHorzPrev = distances.rightHorz;
    distances.rightVertPrev = distances.rightVert;
    distances.minPrev = distances.min;
    // 2. Get the indices for first, middle, and last readings
    uint16_t  rightInd = 0;               // First reading (right)
    uint16_t  frontInd = nLasers / 2;     // Middle reading (front)
    uint16_t  leftInd = nLasers - 1;      // Last reading (left)
    // 3. Find the closes non-nan value for right, front, and left
    distances.leftRay= msg->ranges[leftInd];
    distances.frontRay= msg->ranges[frontInd];
    distances.rightRay = msg->ranges[rightInd];

    while(std::isnan(laser_data.left_distance) && left_idx > 0) {
        laser_data.left_distance = msg->ranges[--left_idx];
    }

    while(std::isnan(laser_data.right_distance) && right_idx < nLasers - 1) {
        laser_data.right_distance = msg->ranges[++right_idx];
    }

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

    // 3c Front
    int j = 1;
    bool odd1 = true;
    while(std::isnan(distances.frontRay)){
        distances.frontRay = msg->ranges[frontInd+j];
        if(odd1){
            odd1 = false;
            j = -(j);
        }
        else{
            odd1 = true;
            j = -(j+1);
        }
    }
    frontInd += j;

    // 4. Calculate Orthogonal (Horz/Vert) for Left and Right
    // 4a Left
    orthogonalizeRay(leftInd, nLasers, distances.leftRay, distances.leftHorz, distances.leftVert);
    // 4b Right
    orthogonalizeRay(rightInd, nLasers, distances.rightRay, distances.rightHorz, distances.rightVert);
    // 5. Calculate min Distance
    distances.min = std::min(std::min(distances.rightRay, distances.frontRay), distances.leftRay);

    ROS_INFO("Left: %.2f m, Front: %.2f m, Right: %.2f m, Min: %.2f m",
             laser_data.left_distance, laser_data.front_distance,
             laser_data.right_distance, laser_data.min_distance);
}
