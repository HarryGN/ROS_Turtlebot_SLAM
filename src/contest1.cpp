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
#include <algorithm>
#include <numeric>

// laserscan to coordinates
#include <vector>
#include <limits>
#include <queue>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

// struct for coordinates store
struct Point {
    float x;
    float y;
    double angle;
};

double posX = 0., posY = 0., yaw = 0.;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
float angular = 0.0;
float linear = 0.0;
float minLaserDist = std::numeric_limits<float>::infinity();
float left_distance = 0.0, right_distance = 0.0, front_distance = 0.0;
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;

Point furthestPoint;  // store the coordinates of the furthest point
double furthestDistance = 0;

// Vector to store VISITED coordinates
std::vector<std::pair<double, double>> positions;

// Vector to store laser scanned coordinates
std::vector<Point> global_scan_points;

// Vector to store 

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

void computeAdvanceCoordinate(float distance, float angle_rad, float posX, float posY, float &targetX, float &targetY){
    float dx = distance * std::cos(angle_rad);
    float dy = distance * std::sin(angle_rad);

    targetX = posX + dx;
    targetY = posY + dy;
}

double calculate_distance(const std::pair<double, double>& p1, const std::pair<double, double>& p2) {
    return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    furthestDistance = 0;
    global_scan_points.clear();  // clear previous scan points

    int nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;

    for (int i = 0; i < nLasers; ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;
        float distance = msg->ranges[i];

        if (!std::isnan(distance)) {
            float targetX, targetY;
            // Calculate global position
            computeAdvanceCoordinate(distance, angle + yaw, posX, posY, targetX, targetY);
            global_scan_points.push_back({targetX, targetY, (angle + yaw)});   // store absolute angle
            ROS_INFO(" Global X: %.2f, Y: %.2f", targetX, targetY);

            // Check if this point is furthest
            if (distance > furthestDistance) {
                furthestDistance = distance;
                furthestPoint.x = targetX;
                furthestPoint.y = targetY;
                furthestPoint.angle = angle + yaw;
            }
        }
    }
    if (furthestDistance > 0){
        ROS_INFO("Furthest Point: Global (%2f, %2f), abs angle: %2f", furthestPoint.x, furthestPoint.y, RAD2DEG(furthestPoint.angle));
    }
}

Point getFurthestPoint(){
    return furthestPoint;
}

const std::vector<Point>& getAllScanPoints(){
    return global_scan_points;
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
        // ROS_INFO("Stored coordinates: (%f, %f)", posX, posY);
    } else {
        // ROS_INFO("Coordinates (%f, %f) already visited, not storing again.", posX, posY);
    }

    // Display the full list of stored coordinates
    /*ROS_INFO("Current list of stored coordinates:");
    for (const auto& coord : positions) {
        ROS_INFO("X: %.2f, Y: %.2f", coord.first, coord.second);
    }
    */
}


// find the corner coordinates with the largest distance between them
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

double get_total_dist() {
    double total_dist = 0.0;

    for (size_t i = 1; i < positions.size(); ++i) {
        total_dist += calculate_distance(positions[i - 1], positions[i]);  // Sum up distances between consecutive points
    }
    
    return total_dist;
}

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

double point_to_line_distance(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end) {
    double normal_length = std::hypot(line_end.first - line_start.first, line_end.second - line_start.second);
    return std::abs((point.first - line_start.first) * (line_end.second - line_start.second) - (point.second - line_start.second) * (line_end.first - line_start.first)) / normal_length;
}

int line_side(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end) {
    double result = (point.first - line_start.first) * (line_end.second - line_start.second) - (point.second - line_start.second) * (line_end.first - line_start.first);
    return (result > 0) ? 1 : (result < 0) ? -1 : 0;
}

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
        // ROS_INFO("Corner 1: (%.2f, %.2f)", corners.first.first, corners.first.second); 
        // ROS_INFO("Corner 2: (%.2f, %.2f)", corners.second.first, corners.second.second);

        bool any_bumper_pressed = false;
        float target_distance = 0.9;


        /*
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
        */
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();

        // End condition for 1 full round
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

