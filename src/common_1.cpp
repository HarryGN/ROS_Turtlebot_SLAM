#include "common_1.h"

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

BumpersStruct bumpers;
LaserScanData laser_data;
OrthogonalDist orthogonal_dist;

#pragma region Global Variable
float angular;
float linear;
float posX = 0.0, posY = 0.0, yaw = 0.0;

float rotationTolerance = Deg2Rad(1);
float kp_r = 1;
float kn_r = 0.7;
float minAngular = 15; // Degrees per second
float maxAngular = 90; // Degrees per second

float navigationTolerance = 0.05;
float kp_n = 0.02;
float kn_n = 0.5;
float minLinear = 0.1;
float maxLinear = 0.45;
bool prev_turn = false;
bool curr_turn = false;

float full_angle = 57;
int nLasers = 0;       // 激光雷达数据的数量
int right_idx = 0;     // 右侧激光索引
int front_idx = 0;     // 前方激光索引
int left_idx = 0;      // 左侧激光索引

std::vector<std::pair<double, double>> positions;

ros::Publisher vel_pub;
#pragma endregion

#pragma region main v
const float target_distance = 0.9;
const float safe_threshold = 1.0;  // Safe distance threshold
const double k = 0.18;   // Scaling factor for angular velocity
const double alpha = 1.8; // Exponential growth/decay rate
const float max_speed = 0.25;  // Max linear speed
const float min_speed = 0.1;   // Min linear speed
float current_x;
float current_y;
float delta_x;
float delta_y;
int corridor_count = 0;
bool wall_following = false;

float prev_left_distance = 0.0, prev_right_distance = 0.0;
float prev_left_idx = 0.0, prev_right_idx = 0.0;

#pragma endregion
