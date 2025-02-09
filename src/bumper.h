#ifndef bumperHeader
#define bumperHeader

#include "common.h"
#include "movement.h"
#include "laser.h"

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

void handleBumperPressed(float turnAngle, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

void checkBumper(geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

#endif