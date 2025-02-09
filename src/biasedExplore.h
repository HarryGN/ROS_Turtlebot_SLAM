#ifndef biasedExploreHeader
#define biasedExploreHeader


#include "common.h"
#include "movement.h"
#include "laser.h"
#include "common.h"

void sweep360(std::vector<std::array<float, 2>> &sweptPoints, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);
void findNextDestination(float posX, float posY, std::vector<std::array<float, 2>> sweptPoints, std::vector<std::array<float, 2>> &visitedPoints, float &nextX, float &nextY);

#endif