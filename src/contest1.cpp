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

//coordinates
#include <algorithm>
#include <numeric>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#define Rad2Deg(rad) ((rad) * 180. / M_PI)
#define Deg2Rad(deg) ((deg) * M_PI / 180.)

// Struct to hold laser scan data
struct LaserScanData {
    float left_distance;
    float front_distance;
    float right_distance;
    float min_distance;
};

struct OrthogonalDist {
    float left_distance;
    float front_distance;
    float right_distance;
};

#pragma region Bumper variable
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

struct BumpersStruct{
    bool leftPressed;
    bool centerPressed;
    bool rightPressed;
    bool anyPressed;
};

BumpersStruct bumpers;

#pragma endregion
// Global Variables
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
bool bumper_event_in_progress = false;
LaserScanData laser_data;
OrthogonalDist orthogonal_dist;
float full_angle = 57;
int nLasers = 0;       // 激光雷达数据的数量
int right_idx = 0;     // 右侧激光索引
int front_idx = 0;     // 前方激光索引
int left_idx = 0;      // 左侧激光索引
// Create a vector to store positions
std::vector<std::pair<double, double>> positions;
ros::Publisher vel_pub;
#pragma endregion

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper] = msg->state;
    bool leftBumperPressed = bumper[kobuki_msgs::BumperEvent::LEFT];
    bool centerBumperPressed = bumper[kobuki_msgs::BumperEvent::CENTER];
    bool rightBumperPressed = bumper[kobuki_msgs::BumperEvent::RIGHT];
    ROS_INFO("BUMPER STATES L/C/R: %u/%u/%u", leftBumperPressed, centerBumperPressed, rightBumperPressed);

    if (leftBumperPressed || centerBumperPressed || rightBumperPressed) {
        // Get the current position from odometry or other source
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map"; // or "odom" depending on your frame

        // Set the position
        pose.pose.position.x = posX;
        pose.pose.position.y = posY;
        pose.pose.position.z = 0.0;

        // Set the orientation
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        // Publish the pose
        pose_pub.publish(pose);

        // Create and publish a marker
        static int marker_id = 0;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "bumper_markers";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = posX;
        marker.pose.position.y = posY;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_pub.publish(marker);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = RAD2DEG(tf::getYaw(msg->pose.pose.orientation));
    // //ROS_INFO("(x,y):(%f,%f) Orint: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
    // ROS_INFO("(x,y):(%f,%f).", posX, posY);
}

void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist) {
    float angle = (float) ind / (float) nLasers * full_angle + 90 - full_angle / 2;
    horz_dist = distance * std::cos(DEG2RAD(angle));
    front_dist = distance * std::sin(DEG2RAD(angle));
    front_dist += 0.05;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    nLasers = static_cast<int>((msg->angle_max - msg->angle_min) / msg->angle_increment);
    // 更新全局变量的值
    right_idx = 0;
    front_idx = nLasers / 2;
    left_idx = nLasers - 1;


    laser_data.left_distance = msg->ranges[left_idx];
    laser_data.front_distance = msg->ranges[front_idx];
    laser_data.right_distance = msg->ranges[right_idx];

    // int i = left_idx-1;
    while(std::isnan(laser_data.left_distance)){
        laser_data.left_distance = msg->ranges[left_idx];
        left_idx--;
    }

    // i = right_idx+1;
    while(std::isnan(laser_data.right_distance)){
        laser_data.right_distance = msg->ranges[right_idx];
        right_idx++;
    }


    int i = 1;
    bool odd = true;

    while(std::isnan(laser_data.front_distance)){
        laser_data.front_distance = msg->ranges[front_idx+i];
        if(odd){
            odd = false;
            i = -(i);
        }
        else{
            odd = true;
            i = -(i+1);
        }
    }

    front_idx += i;

    // Helper function to compute the average of three rays, handling NaNs
    auto avg_range = [&](int idx) -> float {
        float sum = 0.0;
        int count = 0;

        if (idx > 0 && !std::isnan(msg->ranges[idx - 1])) {
            sum += msg->ranges[idx - 1];
            count++;
        }
        if (!std::isnan(msg->ranges[idx])) {
            sum += msg->ranges[idx];
            count++;
        }
        if (idx < nLasers - 1 && !std::isnan(msg->ranges[idx + 1])) {
            sum += msg->ranges[idx + 1];
            count++;
        }

        return (count > 0) ? (sum / count) : std::numeric_limits<float>::quiet_NaN();
    };

    // Store processed values in the struct
    // laser_data.left_distance = avg_range(left_idx);
    // laser_data.front_distance = avg_range(front_idx);
    // laser_data.right_distance = avg_range(right_idx);

    // Compute min distance while ignoring NaNs
    laser_data.min_distance = std::numeric_limits<float>::infinity();
    for (int i = 0; i < nLasers; ++i) {
        if (!std::isnan(msg->ranges[i])) {
            laser_data.min_distance = std::min(laser_data.min_distance, msg->ranges[i]);
        }
    }

    ROS_INFO("Left: %.2f m, Front: %.2f m, Right: %.2f m, Min: %.2f m",
             laser_data.left_distance, laser_data.front_distance,
             laser_data.right_distance, laser_data.min_distance);
}


/// @brief bumper related function/////////////////////////////////////////////////////////////////////////////////////////////
/// @param turnAngle 
/// @param vel 
/// @param vel_pub /
void applyMagnitudeLimits(float &value, float lowerLimit, float upperLimit){
    if(value < 0){
        if(value < -upperLimit){
            value = -upperLimit;
        }
        else if(value > -lowerLimit){
            value = -lowerLimit;
        }
    }

    else if(value > 0){
        if(value > upperLimit){
            value = upperLimit;
        }
        else if(value < lowerLimit){
            value = lowerLimit;
        }
    }
}

float computeAngular(float targetHeading, float currentYaw){
    float angularDeg;

    // Calculate proportional component and then calculate angularDeg based on if it is negative or positive
    float proportional = kp_r*(targetHeading-currentYaw);

    while(proportional > 180){
        proportional -= 360;
    }

    while(proportional < -180){
        proportional += 360;
    }


    if(proportional < 0){
        angularDeg = (float) -1*pow(-1*proportional, kn_r);
    }
    else{
        angularDeg = (float) pow(proportional, kn_r);
    }

    applyMagnitudeLimits(angularDeg, minAngular, maxAngular);

    return Deg2Rad(angularDeg);
}

void rotateToHeading(float targetHeading, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("ROTATE TO HEADING CALLED");
    ros::spinOnce();
    
    float proportional;
    
    while(targetHeading < -180){
        targetHeading += 360;
    }

    while (targetHeading > 180){
        targetHeading -= 360;
    }

    while(abs(targetHeading - yaw) > rotationTolerance){

        angular = computeAngular(targetHeading, yaw);
        linear = 0;

        ros::spinOnce();
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        ROS_INFO("Target/Current Yaw: %f/%f degs | Setpoint: %f degs/s", targetHeading, yaw, Rad2Deg(angular));
    }

    vel.angular.z = 0;
    vel.linear.x = 0;
    vel_pub.publish(vel);

}

void handleBumperPressed(float turnAngle, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    if(bumper_event_in_progress) return;
    bumper_event_in_progress = true;
    ROS_INFO("handleBumperPressed() called...");
    float reverseDistance = 0.2;
    float forwardDistance = reverseDistance / std::cos(Deg2Rad(turnAngle)) * 0.9;

    float exitDistanceThreshold = 0.02;

    // 1. Reverse
    ROS_INFO("handleBumperPressed() | Reversing...");
    float x0 = posX;
    float y0 = posY;
    
    float dx;
    float dy;
    float d = 0;

    while((d-reverseDistance) < exitDistanceThreshold){
        ros::spinOnce();
        if(bumpers.anyPressed){ // Emergency stop if hit again
            ROS_WARN("Emergency stop: bumper re-hit during reverse.");
            linear = 0;
            angular = 0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
            bumper_event_in_progress = false;
            return;
        }
        linear = -0.1;
        angular = 0;
        dx = posX - x0;
        dy = posY - y0;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));



        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }

    
    // 2. Turn
    if(turnAngle == 0){
        if(laser_data.left_distance > laser_data.right_distance){
            turnAngle = 45;
        }

        else{
            turnAngle = -45;
        }

    }
    ROS_INFO("handleBumperPressed() | Turning...");
    rotateToHeading(yaw + turnAngle, vel, vel_pub);

    // 3. Drive Forward
    ROS_INFO("handleBumperPressed() | Advancing...");
    x0 = posX;
    y0 = posY;
    dx = 0;
    dy = 0;
    d = 0;
    
    while((d-forwardDistance) < exitDistanceThreshold){
        ros::spinOnce();
        if(bumpers.anyPressed){ // Emergency stop if hit again
            ROS_WARN("Emergency stop: bumper re-hit during advance.");
            linear = 0;
            angular = 0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
            bumper_event_in_progress = false;
            return;
        }
        linear = 0.1;
        angular = 0;
        dx = posX - x0;
        dy = posY - y0;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));



        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }

    // 4. Turn Back
    ROS_INFO("handleBumperPressed() | Correcting yaw...");
    rotateToHeading(yaw - turnAngle * 0.7, vel, vel_pub);
    ROS_INFO("handleBumperPressed() | END");
    linear = 0;
    angular = 0;    
    angular = 0;    
    
    angular = 0;
    
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
    bumper_event_in_progress = false;
    return;
}

void bumper_handling (geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    if(bumpers.anyPressed){
        if(bumpers.leftPressed){
            handleBumperPressed((float) -45.0, vel, vel_pub);
        }

        else if(bumpers.rightPressed){
            handleBumperPressed((float) 45.0, vel, vel_pub);
        }

        else if(bumpers.centerPressed){
            handleBumperPressed((float) 0.0, vel, vel_pub);
        }
        }
}

/// @brief wall following related function/////////////////////////////////////////////////////////////////////////////////////
/// @param turnAngle 
/// @param vel 
/// @param vel_pub /
// Define an enumeration for wall side
enum WallSide { LEFT, RIGHT };

// Function to move the robot with a given linear and angular velocity
void moveRobot(double linear_x, double angular_z) {
    geometry_msgs::Twist vel_msg;

    // Set the linear and angular velocity
    vel_msg.linear.x = linear_x;
    vel_msg.angular.z = angular_z;

    // Publish the velocity message
    vel_pub.publish(vel_msg);
    ROS_INFO("Publishing velocity command: linear_x = %f, angular_z = %f", linear_x, angular_z);
}

// Function to rotate the robot locally
void rotateRobot(double angular_speed, double duration) {
    geometry_msgs::Twist vel_msg;

    // Set linear velocity to 0 (no forward/backward movement)
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;

    // Set angular velocity (z-axis for rotation)
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = angular_speed; // Positive for counterclockwise, negative for clockwise

    ros::Time start_time = ros::Time::now();
    while ((ros::Time::now() - start_time).toSec() < duration) {
        vel_pub.publish(vel_msg); // Publish the velocity command
        ros::spinOnce(); // Allow ROS to process callbacks
        ros::Duration(0.1).sleep(); // Sleep for a short time to control the loop rate
    }

    // Stop the robot after the duration
    vel_msg.angular.z = 0.0; // Stop rotation
    vel_pub.publish(vel_msg);
    ROS_INFO("Rotation complete. Robot stopped.");
}


// Function to perform wall-following logic
void wallFollowing(WallSide wall_side, bool curr_turn, bool prev_turn, float left_dist, float right_dist, float front_dist, float target_distance, float min_speed, float k, float alpha, geometry_msgs::Twist &vel, ros::Publisher &vel_pub) {
    
    bumper_handling(vel, vel_pub);
    // **Wall-Following Logic**
    if (front_dist > 0.9) {
        if (wall_side == LEFT) {
            // Follow the left wall
            if (left_dist < target_distance) {
                vel.angular.z = -k * (1 - exp(-alpha * left_dist)); // Adjust right
            } else if (left_dist > target_distance) {
                
                vel.angular.z = DEG2RAD(28);  // Adjust left
            } else {
                vel.angular.z = 0.0;  // No adjustment needed
            }
        } else if (wall_side == RIGHT) {
            // Follow the right wall
            if (right_dist < target_distance) {
                vel.angular.z = k * (1 - exp(-alpha * right_dist)); // Adjust left
            } else if (right_dist > target_distance) {
                vel.angular.z = -k * (1 - exp(-alpha * right_dist)); // Adjust right
            } else {
                vel.angular.z = 0.0;  // No adjustment needed
            }
        }
    } 

    else if (front_dist < 0.9 & left_dist < 0.9 & right_dist < 0.9 & curr_turn != prev_turn) {
        // vel.angular.z = -1.57;  // 1.57 radians = 90 degrees
        ROS_INFO("surrounded by three walls");
        rotateRobot(-0.25, 12.56);
        rotateRobot(0, 0);
        curr_turn = true;
        
    }
    else if (front_dist < 0.9 & left_dist < 0.9 & right_dist > 0.9  & curr_turn != prev_turn) {
        // vel.angular.z = -1.57;  // 1.57 radians = 90 degrees
        ROS_INFO("Turn to the right");
        rotateRobot(-0.25, 6.28);
        rotateRobot(0, 0);
        curr_turn = true;
    }
    else if (front_dist < 0.9 & left_dist > 0.9 & right_dist < 0.9 & curr_turn != prev_turn) {
        // vel.angular.z = -1.57;  // 1.57 radians = 90 degrees
        ROS_INFO("Turn to the left");
        rotateRobot(0.25, 6.28);
        rotateRobot(0, 0);
        curr_turn = true;
    }
    else {
        // Obstacle detected in front, slow down and turn
        vel.linear.x = min_speed; // Slow down
        vel.angular.z = (wall_side == LEFT) ? -0.26 : 0.26; // Turn away from the wall
    }
}

/// @brief coordinate related function ///////////
/// @param argc 
/// @param argv 
/// @return //
// Store the current coordinates in the position vector
void get_coord() {
    bool is_visited = false;
    for (const auto& coord : positions) {
        if (std::abs(coord.first - posX) < 0.05 && std::abs(coord.second - posY) < 0.05) {
            is_visited = true;
            break;
        }
    }

    if (!is_visited) {
        positions.push_back(std::make_pair(posX, posY)); // Only store new coordinates
        // ROS_INFO("Stored coordinates: (%f, %f)", posX, posY);
    } else {
        ROS_INFO("Coordinates (%f, %f) already visited, not storing again.", posX, posY);
    }

    // Display the full list of stored coordinates
    ROS_INFO("Current list of stored coordinates:");
    for (const auto& coord : positions) {
        // ROS_INFO("X: %.2f, Y: %.2f", coord.first, coord.second);
    }
}

// Calculate Euclidean distance between two points
double calculate_distance(const std::pair<double, double>& p1, const std::pair<double, double>& p2) {
    return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}

// Function to find the corner coordinates with the largest distance between them
std::pair<std::pair<double, double>, std::pair<double, double>> filter_corner() {
    double max_distance = 0.0;
    std::pair<double, double> corner1, corner2;

    // Iterate through all pairs of coordinates to find the maximum distance
    for (size_t i = 0; i < positions.size(); ++i) {
        for (size_t j = i + 1; j < positions.size(); ++j) {
            double distance = calculate_distance(positions[i], positions[j]);

            if (distance > max_distance) {
                max_distance = distance;
                corner1 = positions[i];
                corner2 = positions[j];
            }
        }
    }

    // Return the two corner coordinates that are the farthest apart
    return std::make_pair(corner1, corner2);
}


// Function to calculate total distance traveled
double get_total_dist() {
    double total_dist = 0.0;

    for (size_t i = 1; i < positions.size(); ++i) {
        total_dist += calculate_distance(positions[i - 1], positions[i]);  // Sum up distances between consecutive points
    }
    
    return total_dist;
}

// Check if the [corner coordinate] is set
// Check if the first cornor coordinate is visited. Robot need to travel at least min_distance
// Need to tune the threshold -----------------------------------------------------------------------------------------------------
bool is_position_visited(double x, double y, double threshold = 1.0, double min_distance = 4) {
    double total_dist = get_total_dist();
    ROS_INFO("Total distance = %f", total_dist);
    bool corner_set = false;

    // corner coordinates set when travel distance >= min_distance
    if (total_dist > min_distance) {
        corner_set = true;
        ROS_INFO("Corner_set Flag = TRUE"); 

        // Read the corner coordinates
        std::pair<std::pair<double, double>, std::pair<double, double>> corners = filter_corner();

        // Check if the current position is within the threshold of the first corner coordinate
        if (std::abs(corners.first.first - x) < threshold && std::abs(corners.first.second - y) < threshold) {
            return true; // Position has been visited
        }
    }
    return false; // Position has not been visited
}

// ---------------------------------------------------------------------------------------------------------------------------------

// Function to calculate the perpendicular distance from a point to a line defined by two points (corner1 and corner2)
double point_to_line_distance(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end) {
    double normal_length = std::hypot(line_end.first - line_start.first, line_end.second - line_start.second);
    return std::abs((point.first - line_start.first) * (line_end.second - line_start.second) - (point.second - line_start.second) * (line_end.first - line_start.first)) / normal_length;
}

// Function to determine the side of the line a point is on
int line_side(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end) {
    double result = (point.first - line_start.first) * (line_end.second - line_start.second) - (point.second - line_start.second) * (line_end.first - line_start.first);
    return (result > 0) ? 1 : (result < 0) ? -1 : 0;
}

// Improved function to find additional diagonal corners based on maximum perpendicular distance
std::vector<std::pair<double, double>> get_all_corners() {
    std::vector<std::pair<double, double>> all_corners;
    std::pair<std::pair<double, double>, std::pair<double, double>> far_corners = filter_corner();

    all_corners.push_back(far_corners.first);
    all_corners.push_back(far_corners.second);

    // Define variables for additional corners with max distances
    double max_distance_1 = 0.0, max_distance_2 = 0.0;
    std::pair<double, double> corner_3, corner_4;
    int side_first = line_side(positions.front(), far_corners.first, far_corners.second);

    for (const auto& coord : positions) {
        double dist = point_to_line_distance(coord, far_corners.first, far_corners.second);
        int side = line_side(coord, far_corners.first, far_corners.second);

        // Ensure we're selecting points from opposite sides
        if (side != side_first && dist > max_distance_1) {
            max_distance_1 = dist;
            corner_3 = coord;
        } else if (side == side_first && dist > max_distance_2) {
            max_distance_2 = dist;
            corner_4 = coord;
        }
    }

    all_corners.push_back(corner_3);
    all_corners.push_back(corner_4);

    return all_corners;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odomCallback);

    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

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

    // 全局变量：存储上一帧的左/右激光读数
    float prev_left_distance = 0.0, prev_right_distance = 0.0;
    float prev_left_idx = 0.0, prev_right_idx = 0.0;

    while (ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        
        get_coord();  //store positions
        std::pair<std::pair<double, double>, std::pair<double, double>> corners = filter_corner();
        // Print corner coord
        ROS_INFO("Corner 1: (%.2f, %.2f)", corners.first.first, corners.first.second); 
        ROS_INFO("Corner 2: (%.2f, %.2f)", corners.second.first, corners.second.second);



        float front_dist = std::isnan(laser_data.front_distance) ? safe_threshold : laser_data.front_distance;
        float left_dist = std::isnan(laser_data.left_distance) ? safe_threshold : laser_data.left_distance;
        float right_dist = std::isnan(laser_data.right_distance) ? safe_threshold : laser_data.right_distance;

        //initialize turn indicator for current loop
        bool curr_turn = false;
        // Speed Zone check
        float closest_obstacle = std::min({front_dist, left_dist, right_dist});

        // Dynamic vel control
        if (closest_obstacle >= safe_threshold) {
            vel.linear.x = max_speed;  // 没有障碍物时跑最快
        } else {
            vel.linear.x = min_speed + (max_speed - min_speed) * ((closest_obstacle - target_distance) / (safe_threshold - target_distance));
            vel.linear.x = std::max(static_cast<double>(min_speed), std::min(static_cast<double>(max_speed), vel.linear.x));
        }

        // **检测通道**
        float corridor_threshold = 0.6;   // 设定通道触发阈值
        float max_distance_change = 1.5;  // 设定最大突变距离
        float min_rotation = 30.0;        // 最小旋转角度
        float max_rotation = 90.0;        // 最大旋转角度

        float left_change = left_dist - prev_left_distance;
        float right_change = right_dist - prev_right_distance;

        if (left_change < 0.1 || right_change < 0.1){
            wall_following = true;
        }

        if (left_change > corridor_threshold & wall_following) {
            // Calculate rotation angle (clamped between min_rotation and max_rotation)
            float rotation_angle_deg = min_rotation + (left_change - corridor_threshold) * (max_rotation - min_rotation) / (max_distance_change - corridor_threshold);
            rotation_angle_deg = std::min(max_rotation, std::max(min_rotation, rotation_angle_deg)); // Clamp the angle
            float rotation_angle_rad = DEG2RAD(rotation_angle_deg); // Convert to radians

            ROS_WARN("Detected corridor on the LEFT! Turning %.1f°", rotation_angle_deg);
            
            vel.angular.z = 0.0;  // No adjustment needed
            // Orthogonalize the laser data
            orthogonalizeRay(prev_left_idx, nLasers, prev_left_distance, orthogonal_dist.left_distance, orthogonal_dist.front_distance);

            // Initialize current position if this is the first corridor detection
            if (corridor_count < 1) {
                current_x = posX;
                current_y = posY;
                corridor_count += 1;
            }

            moveRobot(orthogonal_dist.front_distance, 0);
            ROS_WARN("front distance move %.1f°", orthogonal_dist.front_distance);
            wall_following = false;
        }

        else if (right_change > corridor_threshold & wall_following) {
            // 计算旋转角度 (限制 30° ~ 90°)
            float rotation_angle_deg = min_rotation + (right_change - corridor_threshold) * (max_rotation - min_rotation) / (max_distance_change - corridor_threshold);
            rotation_angle_deg = std::min(max_rotation, std::max(min_rotation, rotation_angle_deg));
            float rotation_angle_rad = DEG2RAD(rotation_angle_deg);

            ROS_WARN("Detected corridor on the RIGHT! Turning %.1f°", rotation_angle_deg);
            wall_following = false;
        }

        else {
            // Choose the wall to follow (LEFT or RIGHT)
            WallSide wall_side = LEFT; // Change to RIGHT to follow the right wall
            // ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

            // Call the wall-following function
            wallFollowing(wall_side, curr_turn, prev_turn, left_dist, right_dist, front_dist, 
            target_distance, min_speed, k, alpha, vel, vel_pub);
        }

        // **存储当前激光读数，供下一次循环比较**
        prev_left_distance = left_dist;
        prev_right_distance = right_dist;
        prev_left_idx = left_idx;
        prev_right_idx = right_idx;
        prev_turn = curr_turn;

        // 发布速度指令
        vel_pub.publish(vel);

        // 更新时间
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - start
        ).count();

        loop_rate.sleep();
        ////////////////////////////////////////////////////////////////coordinate break check////////////////////////////////////////////////////
        // Check if the robot has returned to a previous position
        // Then start zig-zag. Need to incorporate
        if (is_position_visited(posX, posY)) {
            ROS_INFO("Robot has completed a round and returned to previous position.");
            
            //get_all_corners
            std::vector<std::pair<double, double>> corners = get_all_corners();
            
            // Printing out the corners for debugging or monitoring
            ROS_INFO("Detected Corners:");
            for (const auto& corner : corners) {
                ROS_INFO("Corner: (X: %.2f, Y: %.2f)", corner.first, corner.second);
            }

	        break;  // Stop the loop if visited position
        }
    }
    return 0;
}
