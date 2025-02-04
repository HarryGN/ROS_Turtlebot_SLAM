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

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    bumper[msg->bumper] = msg->state;
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
            i = -(i+1)
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odomCallback);

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

    // 全局变量：存储上一帧的左/右激光读数
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

        if (left_change > corridor_threshold) {
            // 计算旋转角度 (限制 30° ~ 90°)
            float rotation_angle_deg = min_rotation + (left_change - corridor_threshold) * (max_rotation - min_rotation) / (max_distance_change - corridor_threshold);
            rotation_angle_deg = std::min(max_rotation, std::max(min_rotation, rotation_angle_deg));
            float rotation_angle_rad = DEG2RAD(rotation_angle_deg);

            ROS_WARN("Detected corridor on the LEFT! Turning %.1f°", rotation_angle_deg);
            // vel.linear.x = 0.0;  use bumper data to continues check opening
            // vel.angular.z = rotation_angle_rad; // 左转
        }
        else if (right_change > corridor_threshold) {
            // 计算旋转角度 (限制 30° ~ 90°)
            float rotation_angle_deg = min_rotation + (right_change - corridor_threshold) * (max_rotation - min_rotation) / (max_distance_change - corridor_threshold);
            rotation_angle_deg = std::min(max_rotation, std::max(min_rotation, rotation_angle_deg));
            float rotation_angle_rad = DEG2RAD(rotation_angle_deg);

            ROS_WARN("Detected corridor on the RIGHT! Turning %.1f°", rotation_angle_deg);
            // vel.linear.x = 0.0;   use bumper data to continues check opening
            // vel.angular.z = -rotation_angle_rad; // 右转
        }

        else {
            // **正常扶墙逻辑**
            if (front_dist > 1.0) {
                if (left_dist < target_distance) {
                    vel.angular.z = -k * (1 - exp(-alpha * left_dist)); // 向右调整
                } 
                else if (left_dist > target_distance) {
                    vel.angular.z = k * (1 - exp(-alpha * left_dist));  // 向左调整
                } 
                else {
                    vel.angular.z = 0.0;  
                }
            } 
            else {
                vel.linear.x = min_speed; // 减速
                vel.angular.z = -0.26;    // 右转避障
            }
        }

        // **存储当前激光读数，供下一次循环比较**
        prev_left_distance = left_dist;
        prev_right_distance = right_dist;

        // 发布速度指令
        vel_pub.publish(vel);

        // 更新时间
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - start
        ).count();

        loop_rate.sleep();
    }

    return 0;
}
