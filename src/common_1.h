#ifndef common_1Header
#define common_1Header

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#define Rad2Deg(rad) ((rad) * 180. / M_PI)
#define Deg2Rad(deg) ((deg) * M_PI / 180.)

#pragma region Global
extern float angular;
extern float linear;
extern float posX, posY, yaw;

extern float rotationTolerance;
extern float kp_r, kn_r;
extern float minAngular, maxAngular;

extern float navigationTolerance;
extern float kp_n, kn_n;
extern float minLinear, maxLinear;
extern bool prev_turn, curr_turn;
extern float full_angle;

extern std::vector<std::pair<double, double>> positions;
extern ros::Publisher vel_pub;
#pragma endregion



#pragma region bumper
// uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
extern uint8_t bumper[3];
struct BumpersStruct{
    bool leftPressed;
    bool centerPressed;
    bool rightPressed;
    bool anyPressed;
};
extern BumpersStruct bumpers;
#pragma endregion

#pragma region laser
struct LaserScanData {
    float left_distance;
    float front_distance;
    float right_distance;
    float min_distance;
};

extern LaserScanData laser_data;

struct OrthogonalDist {
    float left_distance;
    float front_distance;
    float right_distance;
};

extern OrthogonalDist orthogonal_dist;

#pragma endregion

#endif
