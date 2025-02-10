<<<<<<< HEAD
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
=======
#ifndef laser_1Header
#define laser_1Header

#include "common_1.h"

void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist);

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);



>>>>>>> 768b77a

#endif