#ifndef laserHeader
#define laserHeader

#include "common.h"

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist);

#endif