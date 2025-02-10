<<<<<<< HEAD
#ifndef bumper1Header
#define bumper1Header

#include "common.h"
#include "laser_1.h"

=======
#ifndef bumper_1Header
#define bumper_1Header

#include "common_1.h"

// functions
>>>>>>> 768b77a
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

void applyMagnitudeLimits(float &value, float lowerLimit, float upperLimit);

float computeAngular(float targetHeading, float currentYaw);

void rotateToHeading(float targetHeading, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

void handleBumperPressed(float turnAngle, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

void bumper_handling (geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

<<<<<<< HEAD
=======

>>>>>>> 768b77a
#endif