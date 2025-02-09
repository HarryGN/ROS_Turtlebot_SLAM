#ifndef movementHeader
#define movementHeader

#include "common.h"
#include "laser.h"
#include "bumper.h"


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

float computeAngular(float targetHeading, float currentYaw);

float computeLinear(float tgtX, float tgtY, float posX, float posY);

void computeAdvanceCoordinate(float distance, float yaw, float posX, float posY, float &targetX, float &targetY);

void computeTargetCoordinate(float distance, float angle, float posX, float posY, float yaw, float &tgtX, float &tgtY);

void rotateToHeading(float targetHeading, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

void navigateToPosition(float tgtX, float tgtY, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

void navigateToPositionSmart(float tgtX, float tgtY, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

#endif
