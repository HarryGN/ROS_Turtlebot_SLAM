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

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

double posX = 0., posY = 0., yaw = 0.;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
float angular = 0.0;
float linear = 0.0;
float minLaserDist = std::numeric_limits<float>::infinity();

float left_distance = 0.0, right_distance = 0.0, front_distance = 0.0;
// float maxLaserDist = std::numeric_limits<float>::();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;

// Create a vector to store positions
std::vector<std::pair<double, double>> positions;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg->state;
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("(x,y):(%f,%f) Orint: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
    ROS_INFO("(x,y):(%f,%f).", posX, posY);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    minLaserDist = std::numeric_limits<float>::infinity();
    // maxLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);

    // Get the indices for first, middle, and last readings
    int right_idx = 0;                 // First reading (left)
    int front_idx = nLasers / 2;      // Middle reading (front)
    int left_idx = nLasers - 1;      // Last reading (right)
    
    right_distance = msg->ranges[right_idx];
    front_distance = msg->ranges[front_idx];
    left_distance = msg->ranges[left_idx];

    // Log the results
    //ROS_INFO("Left (first) distance: %.2f m", left_distance);
    //ROS_INFO("Front (middle) distance: %.2f m", front_distance);
    //ROS_INFO("Right (last) distance: %.2f m", right_distance);
    
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
             std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }

    //ROS_INFO("Min distance: %i", minLaserDist);
}

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
        ROS_INFO("Stored coordinates: (%f, %f)", posX, posY);
    } else {
        ROS_INFO("Coordinates (%f, %f) already visited, not storing again.", posX, posY);
    }

    // Display the full list of stored coordinates
    ROS_INFO("Current list of stored coordinates:");
    for (const auto& coord : positions) {
        ROS_INFO("X: %.2f, Y: %.2f", coord.first, coord.second);
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
    ROS_INFO("Total distance = %f", total_dist);
    return total_dist;
}




// Check if the same place is visited. Robot need to travel at least min_distance
// Need to tune the threshold -----------------------------------------------------------------------------------------------------
// Modify !!!!!
bool is_position_visited(double x, double y, double threshold = 1.0, double min_distance = 4) {
    double total_dist = get_total_dist();

    // Only check for revisiting if the robot has moved at least 'min_distance'
    if (total_dist > min_distance) {
        // Check if the current position is within the threshold of any stored position
        for (const auto& coord : positions) {
            if (std::abs(coord.first - x) < threshold && std::abs(coord.second - y) < threshold) {
                return true; // Position has been visited
            }
        }
    }

    return false; // Position has not been visited
}

// ---------------------------------------------------------------------------------------------------------------------------------



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback); 

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Rate loop_rate(10);

    // // Create a timer to call store_coordinates every second 
    // ros::Timer timer = nh.createTimer(ros::Duration(1.0), store_coordinates);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        // ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, maxLaserRange);
        // Check if any of the bumpers were pressed.

        get_coord();  //store positions
        std::pair<std::pair<double, double>, std::pair<double, double>> corners = filter_corner();
        // Print corner coord
        ROS_INFO("Corner 1: (%.2f, %.2f)", corners.first.first, corners.first.second); 
        ROS_INFO("Corner 2: (%.2f, %.2f)", corners.second.first, corners.second.second);

        bool any_bumper_pressed = false;
        float target_distance = 0.9;
        if (front_distance > 1.0 && !std::isnan(front_distance) && !std::isnan(left_distance) && !std::isnan(right_distance)) {
    
            const double k = 0.15;   // Scaling factor for angular velocity
            const double alpha = 1.5; // Exponential growth/decay rate

            if (left_distance < target_distance) {
                angular = -k * (1-exp(-alpha * left_distance)); // Exponential decay for left turns
                linear = 0.1;                                   // Set a constant forward speed
            } 
            else if (left_distance > target_distance) {
                angular = k * (1-exp(-alpha * left_distance));  // Exponential decay for right turns
                linear = 0.1;                                   // Set a constant forward speed
            } 
            else {
                angular = 0.0;                                  // No angular adjustment
                linear = 0.1;                                   // Maintain forward speed
            }
        }
        else {
            linear = 0.04;
            angular = -0.26;                     // Rotate in place to adjust to right
        }
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();

        // Check if the robot has returned to a previous position
        // Then start zig-zag. Need to incorporate
        if (is_position_visited(posX, posY)) {
            ROS_INFO("Robot has completed a round and returned to previous position.");
	        break;  // Stop the loop if visited position
        }
    }

    return 0;
}

