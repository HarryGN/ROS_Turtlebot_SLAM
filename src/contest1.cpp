#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <unordered_map>
#include <cmath>
#include <limits>
#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// Struct to hold laser scan data
struct LaserScanData {
    float left_distance;
    float front_distance;
    float right_distance;
    float min_distance;
};

// Global Variables
double posX = 0., posY = 0., yaw = 0.;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
LaserScanData laser_data;
ros::Publisher pose_pub;
ros::Publisher marker_pub;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
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
        static int marker_id = 0; // Static variable to keep track of marker IDs
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // or "odom" depending on frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "bumper_markers";
        marker.id = marker_id++; // Increment marker ID for each new marker
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = posX;
        marker.pose.position.y = posY;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.scale.x = 0.2; // Size of the marker
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; // Alpha
        marker.color.r = 1.0; // Red
        marker.color.g = 0.0; // Green
        marker.color.b = 0.0; // Blue

        marker_pub.publish(marker);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int nLasers = static_cast<int>((msg->angle_max - msg->angle_min) / msg->angle_increment);
    int right_idx = 0;
    int front_idx = nLasers / 2;
    int left_idx = nLasers - 1;

    laser_data.left_distance = msg->ranges[left_idx];
    laser_data.front_distance = msg->ranges[front_idx];
    laser_data.right_distance = msg->ranges[right_idx];

    while(std::isnan(laser_data.left_distance)){
        laser_data.left_distance = msg->ranges[left_idx];
        left_idx--;
    }

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "contest1");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    const float target_distance = 0.9;
    const float safe_threshold = 1.0;  // Safe distance threshold
    const double k = 0.15;   // Scaling factor for angular velocity
    const double alpha = 1.5; // Exponential growth/decay rate
    const float max_speed = 0.25;  // Max linear speed
    const float min_speed = 0.1;   // Min linear speed

    // Global variables: store previous frame's left/right laser readings
    float prev_left_distance = 0.0, prev_right_distance = 0.0;

    while (ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        float front_dist = std::isnan(laser_data.front_distance) ? safe_threshold : laser_data.front_distance;
        float left_dist = std::isnan(laser_data.left_distance) ? safe_threshold : laser_data.left_distance;
        float right_dist = std::isnan(laser_data.right_distance) ? safe_threshold : laser_data.right_distance;

        // Speed Zone check
        float closest_obstacle = std::min({front_dist, left_dist, right_dist});

        // Dynamic vel control
        if (closest_obstacle >= safe_threshold) {
            vel.linear.x = max_speed;  // No obstacles, move at max speed
        } else {
            vel.linear.x = min_speed + (max_speed - min_speed) * ((closest_obstacle - target_distance) / (safe_threshold - target_distance));
            vel.linear.x = std::max(static_cast<double>(min_speed), std::min(static_cast<double>(max_speed), vel.linear.x));
        }

        // Corridor detection
        float corridor_threshold = 0.6;   // Set corridor trigger threshold
        float max_distance_change = 1.5;  // Set max change distance
        float min_rotation = 30.0;        // Min rotation angle
        float max_rotation = 90.0;        // Max rotation angle

        float left_change = left_dist - prev_left_distance;
        float right_change = right_dist - prev_right_distance;

        if (left_change > corridor_threshold) {
            // Calculate rotation angle (limit 30° ~ 90°)
            float rotation_angle_deg = min_rotation + (left_change - corridor_threshold) * (max_rotation - min_rotation) / (max_distance_change - corridor_threshold);
            rotation_angle_deg = std::min(max_rotation, std::max(min_rotation, rotation_angle_deg));
            float rotation_angle_rad = DEG2RAD(rotation_angle_deg);

            ROS_WARN("Detected corridor on the LEFT! Turning %.1f°", rotation_angle_deg);
            // vel.linear.x = 0.0;  use bumper data to continues check opening
            // vel.angular.z = rotation_angle_rad; // Turn left
        }
        else if (right_change > corridor_threshold) {
            // Calculate rotation angle (limit 30° ~ 90°)
            float rotation_angle_deg = min_rotation + (right_change - corridor_threshold) * (max_rotation - min_rotation) / (max_distance_change - corridor_threshold);
            rotation_angle_deg = std::min(max_rotation, std::max(min_rotation, rotation_angle_deg));
            float rotation_angle_rad = DEG2RAD(rotation_angle_deg);

            ROS_WARN("Detected corridor on the RIGHT! Turning %.1f°", rotation_angle_deg);
            // vel.linear.x = 0.0;   use bumper data to continues check opening
            // vel.angular.z = -rotation_angle_rad; // Turn right
        }

        else {
            // Normal wall-following logic
            if (front_dist > 1.0) {
                if (left_dist < target_distance) {
                    vel.angular.z = -k * (1 - exp(-alpha * left_dist)); // Adjust right
                } 
                else if (left_dist > target_distance) {
                    vel.angular.z = k * (1 - exp(-alpha * left_dist));  // Adjust left
                } 
                else {
                    vel.angular.z = 0.0;  
                }
            } 
            else {
                vel.linear.x = min_speed; // Slow down
                vel.angular.z = -0.26;    // Turn right to avoid obstacle
            }
        }

        // Store current laser readings for next loop comparison
        prev_left_distance = left_dist;
        prev_right_distance = right_dist;

        // Publish velocity command
        vel_pub.publish(vel);

        // Update elapsed time
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - start
        ).count();

        loop_rate.sleep();
    }

    return 0;
}