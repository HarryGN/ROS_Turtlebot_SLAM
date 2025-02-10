#ifndef wallFollowingHeader
#define wallFollowingHeader

#include "common.h"
#include "bumper.h"
#include "movement.h"

enum WallSide { LEFT, RIGHT };


void get_coord();

double calculate_distance(const std::pair<double, double>& p1, const std::pair<double, double>& p2);

std::pair<std::pair<double, double>, std::pair<double, double>> filter_corner();

double get_total_dist();

bool is_position_visited(double x, double y, double threshold = 1.0, double min_distance = 4);

double point_to_line_distance(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end);

int line_side(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end);

std::vector<std::pair<double, double>> get_all_corners();

void moveRobot(double linear_x, double angular_z, geometry_msgs::Twist &vel_msg, ros::Publisher &vel_pub);

void rotateRobot(double angular_speed, double duration, geometry_msgs::Twist &vel_msg, ros::Publisher &vel_pub);

void wallFollowing(WallSide wall_side, DistancesStruct distances, bool curr_turn, bool prev_turn, float left_dist, float right_dist, float front_dist, float target_distance, float min_speed, float k, float alpha, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

void bumper_handling (geometry_msgs::Twist &vel, ros::Publisher &vel_pub);

void handleBumperPressed2(float turnAngle, geometry_msgs::Twist &vel, ros::Publisher &vel_pub);





#endif