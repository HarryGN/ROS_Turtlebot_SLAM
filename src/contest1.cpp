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
#include <algorithm>
#include <numeric>

// Defined Headers
#include "updateCoordinate.h"
#include "laser.h"
#include "bumper.h"
#include "movement.h"
#include "common.h"

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#define Rad2Deg(rad) ((rad) * 180. / M_PI)
#define Deg2Rad(deg) ((deg) * M_PI / 180.)

BumpersStruct bumpers;
LaserScanData laser_data;
OrthogonalDist orthogonal_dist;


// Create a vector to store positions
std::vector<std::pair<double, double>> positions;

ros::Publisher vel_pub;


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
