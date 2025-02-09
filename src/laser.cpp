#include "laser.h"

uint16_t nLasers;
float fullAngle = 57.0;

DistancesStruct distances;


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

void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist){
    float angle = (float) ind / (float) nLasers * fullAngle + 90 - fullAngle/2;
    horz_dist = std::abs(distance * std::cos(Deg2Rad(angle)));
    front_dist = std::abs(distance * std::sin(Deg2Rad(angle)));
}