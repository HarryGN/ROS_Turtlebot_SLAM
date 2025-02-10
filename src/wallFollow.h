#ifndef wallFollowHeader
#define wallFollowHeader

#include "bumper_1.h"
#include "common.h"

#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#define Rad2Deg(rad) ((rad) * 180. / M_PI)
#define Deg2Rad(deg) ((deg) * M_PI / 180.)


enum WallSide { LEFT, RIGHT };

void moveRobot(double linear_x, double angular_z, ros::Publisher &vel_pub);

void rotateRobot(double angular_speed, double duration, ros::Publisher &vel_pub);

void wallFollowing(WallSide wall_side, bool curr_turn, bool prev_turn, float left_dist, float right_dist, float front_dist, float target_distance, float min_speed, float k, float alpha, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);




#endif