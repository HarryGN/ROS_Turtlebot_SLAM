#ifndef LASER_H
#define LASER_H

#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <limits>
#include "common.h"

// Structures
struct LaserScanData {
    float left_distance;
    float front_distance;
    float right_distance;
    float min_distance;
};

struct OrthogonalDist {
    float left_distance;
    float front_distance;
    float right_distance;
};

// Function declarations
void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

#endif
