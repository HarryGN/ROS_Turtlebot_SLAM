#ifndef commonHeader
#define commonHeader

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>
#include <thread>

#include <math.h>   
#include <string>

#include <vector>
#include <array>

#define Rad2Deg(rad) ((rad) * 180. / M_PI)
#define Deg2Rad(deg) ((deg) * M_PI / 180.)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)


#pragma region Bumper

extern uint8_t bumper[3];

struct BumpersStruct{
    bool leftPressed;
    bool centerPressed;
    bool rightPressed;
    bool anyPressed;
};

extern BumpersStruct bumpers;

// for bumper_1
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

#pragma endregion

#pragma region Laser
struct DistancesStruct{
    float leftRay;
    float leftRayPrev;
    float leftHorz;
    float leftHorzPrev;
    float leftVert;
    float leftVertPrev;

    float frontRay;
    float frontRayPrev;

    float rightRay;
    float rightRayPrev;
    float rightHorz;
    float rightHorzPrev;
    float rightVert;
    float rightVertPrev;

    float min;
    float minPrev;
};

extern DistancesStruct distances;

#pragma endregion

#pragma region Movement

extern float angular;
extern float linear;
extern float posX, posY, yaw;

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

#pragma endregion

#pragma region Functions

float absPow(float base, float exp);

void applyMagnitudeLimits(float &value, float lowerLimit, float upperLimit);

float distanceBetween(float x1, float y1, float x2, float y2);

void wrapIntegerIndexAroundRange(int &index, int start, int end);
#pragma endregion


#endif


