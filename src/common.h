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


#pragma region Bumper

extern uint8_t bumper[3];

struct BumpersStruct{
    bool leftPressed;
    bool centerPressed;
    bool rightPressed;
    bool anyPressed;
};

extern BumpersStruct bumpers;

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

#pragma endregion

#pragma region Functions

float absPow(float base, float exp);

void applyMagnitudeLimits(float &value, float lowerLimit, float upperLimit);

float distanceBetween(float x1, float y1, float x2, float y2);

void wrapIntegerIndexAroundRange(int &index, int start, int end);
#pragma endregion


#endif
