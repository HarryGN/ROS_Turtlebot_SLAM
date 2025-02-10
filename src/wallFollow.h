#ifndef wallHeader
#define wallHeader

#include "common_1.h"
#include "bumper_1.h"

enum WallSide { LEFT, RIGHT };

void moveRobot(double linear_x, double angular_z);

void rotateRobot(double angular_speed, double duration);

void wallFollowing(WallSide wall_side, bool curr_turn, bool prev_turn, float left_dist, float right_dist, float front_dist, float target_distance, float min_speed, float k, float alpha, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);


#endif