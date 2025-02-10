#ifndef laser_1Header
#define laser_1Header

#include "common_1.h"

void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist);

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);




#endif