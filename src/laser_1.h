#ifndef laser1Header
#define laser1Header
#include "common.h"

// Struct to hold laser scan data
struct LaserScanData {
    float left_distance;
    float front_distance;
    float right_distance;
    float min_distance;
};

extern LaserScanData laser_data;

struct OrthogonalDist {
    float left_distance;
    float front_distance;
    float right_distance;
};

extern OrthogonalDist orthogonal_dist;

void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist);

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

#endif