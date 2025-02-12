#ifndef bumperHeader
#define bumperHeader

#include "common.h"
#include "movement.h"
#include "laser.h"

// Include additional message types for RViz markers and poses.
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

// Declare external publishers for marker
extern ros::Publisher pose_pub;
extern ros::Publisher marker_pub;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

void handleBumperPressed(float turnAngle, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

void checkBumper(geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

#endif