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

float rotationTolerance = DEG2RAD(1);
float kp_r = 1;
float kn_r = 0.7;
float minAngular = 15; // Degrees per second
float maxAngular = 90; // Degrees per second

float navigationTolerance = 0.05;
float kp_n = 0.02;
float kn_n = 0.5;
float minLinear = 0.1;
float maxLinear = 0.45;

int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;

#pragma region Setup Coordinates
float offset = 0.8;
Point furthestPoint;  // store the coordinates of the furthest point
double furthestDistance = 0;

// Vector to store VISITED coordinates
std::vector<std::pair<double, double>> positions;

// Vector to store laser scanned coordinates
std::vector<Point> global_scan_points;
#pragma endregion

// BumpersStruct bumpers;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    bumper[msg->bumper] = msg->state;
    bumpers.leftPressed = bumper[kobuki_msgs::BumperEvent::LEFT];
    bumpers.centerPressed = bumper[kobuki_msgs::BumperEvent::CENTER];
    bumpers.rightPressed = bumper[kobuki_msgs::BumperEvent::RIGHT];

    bumpers.anyPressed = bumpers.leftPressed || bumpers.centerPressed || bumpers.rightPressed;

    ROS_INFO("BUMPER STATES L/C/R: %u/%u/%u", bumpers.leftPressed, bumpers.centerPressed, bumpers.rightPressed);
}
void handleBumperPressed(float turnAngle, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
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

        linear = -0.1;
        angular = 0;

        dx = posX-x0;
        dy = posY-y0;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }


    // 2. Turn
    if(turnAngle == 0){ // If center bumper was pressed this is called
        if(distances.leftRay > distances.rightRay){
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

        linear = 0.1;
        angular = 0;

        dx = posX-x0;
        dy = posY-y0;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));



        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
    }

    // 4. Turn Back
    ROS_INFO("handleBumperPressed() | Correcting yaw...");
    rotateToHeading(yaw - turnAngle, vel, vel_pub);



    ROS_INFO("handleBumperPressed() | END");
    linear = 0;
    angular = 0;

    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    return;

}

void checkBumper(geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
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

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("(x,y):(%f,%f) Orint: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
    ROS_INFO("(x,y):(%f,%f).", posX, posY);
}

#pragma region Get Coordinate
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
            // ROS_INFO(" Global X: %.2f, Y: %.2f", targetX, targetY);

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
        ROS_INFO("Furthest Point P: Global (%2f, %2f), abs angle: %2f", furthestPoint.x, furthestPoint.y, RAD2DEG(furthestPoint.angle));
    }
}

Point getFurthestPoint(){
    return furthestPoint;
}

const std::vector<Point>& getAllScanPoints(){
    return global_scan_points;
}


Point get_offset_target(double posX, double posY, Point furthestPoint, float offset) {
    Point target;

    float vectorX = furthestPoint.x - posX;
    float vectorY = furthestPoint.y - posY;

    double distance = calculate_distance({posX, posY}, {furthestPoint.x, furthestPoint.y});

    float normalizedX = vectorX / distance;
    float normalizedY = vectorY / distance;

    // offset from P
    target.x = furthestPoint.x - normalizedX * offset;
    target.y = furthestPoint.y - normalizedY * offset;
    target.angle = furthestPoint.angle; 

    ROS_INFO("Target Point Q: Global (%2f,%2f), abs angle: %2f", target.x, target.y, RAD2DEG(target.angle));

    return target;
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
#pragma endregion

#pragma region Get Corners
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
#pragma endregion
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

float computeLinear(float tgtX, float tgtY, float posX, float posY){
    float dx = tgtX-posX;
    float dy = tgtY-posY;
    float d = (float) sqrt(pow(dx, 2) + pow(dy, 2));
    float localLinear = (float) pow(kp_n*d, kn_n);
    applyMagnitudeLimits(localLinear, minLinear, maxLinear);

    return localLinear;
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

    return DEG2RAD(angularDeg);
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

        //ROS_INFO("Target/Current Yaw: %f/%f degs | Setpoint: %f degs/s", targetHeading, yaw, Rad2Deg(angular));
    }

    vel.angular.z = 0;
    vel.linear.x = 0;
    vel_pub.publish(vel);

}

void navigateToPosition(float tgtX, float tgtY, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("navigateToPosition() called with target(%.2f, %.2f)...", tgtX, tgtY);
    ros::spinOnce();

    int counter = 0;
    int bumperHits = 0;
    int bumperHitsLimit = 3;
    
    float dx = tgtX-posX;
    float dy = tgtY-posY;
    float d = (float) sqrt(pow(dx, 2) + pow(dy, 2));

    // Set and rotate to initial heading
    float targetHeading = Rad2Deg(atan2(dy, dx));
    rotateToHeading(targetHeading, vel, vel_pub);

    // While loop until robot gets there
    while(d > navigationTolerance){
        ros::spinOnce();
        dx = tgtX-posX;
        dy = tgtY-posY;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));

        if(bumpers.anyPressed && (bumperHits >= bumperHitsLimit || d < navigationBumperExitTolerance)){
            checkBumper(vel, vel_pub);
            return;
        }

        else if (bumpers.anyPressed){
            bumperHits ++;
            ROS_INFO("WARNING: BUMPER HITS: %d", bumperHits);
            checkBumper(vel, vel_pub);
            
        }
        

        linear = computeLinear(tgtX, tgtY, posX, posY);

        targetHeading = Rad2Deg(atan2(dy, dx));
        minAngular = 0;
        angular = computeAngular(targetHeading, yaw);
        minAngular = 15;

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        counter ++;
        
        // if(counter%500000 == 0){
        //     ROS_INFO("Tgt X/Y: %f/%f | Pos X/Y: %f/%f | Lin/Ang: %f/%f", tgtX, tgtY, posX, posY, linear, Rad2Deg(angular));
        // }
        
    }

    linear = 0;
    angular = 0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    ROS_INFO("...navigateToPosition completed.");
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

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        get_coord();  //store positions

        bool any_bumper_pressed = false;
        float target_distance = 0.9;

        Point furthestPoint = getFurthestPoint();
        Point targetPoint = get_offset_target(posX, posY, furthestPoint, offset);
        navigateToPosition(targetPoint.x, targetPoint.y, vel, vel_pub);
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

