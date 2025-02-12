#include "wallFollowing.h"

std::vector<std::pair<double, double>> positions;

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
    // ROS_INFO("Current list of stored coordinates:");
    // for (const auto& coord : positions) {
    //     // ROS_INFO("X: %.2f, Y: %.2f", coord.first, coord.second);
    // }
}

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
bool is_position_visited(double x, double y, double threshold, double min_distance) {
    double total_dist = get_total_dist();
    // ROS_INFO("Total distance = %f", total_dist);
    bool corner_set = false;

    // corner coordinates set when travel distance >= min_distance
    if (total_dist > min_distance) {
        corner_set = true;
        // ROS_INFO("Corner_set Flag = TRUE"); 

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

void moveRobot(double linear_x, double angular_z, geometry_msgs::Twist &vel_msg, ros::Publisher &vel_pub) {
    // 设定固定速度 0.1 m/s
    double speed = 0.1;  
    double duration = linear_x / speed; // 计算所需时间

    // 设置线速度和角速度
    vel_msg.linear.x = speed;
    vel_msg.angular.z = angular_z;

    ros::Time start_time = ros::Time::now();
    while ((ros::Time::now() - start_time).toSec() < duration) {
        vel_pub.publish(vel_msg); // 发布速度指令
        ros::spinOnce(); // 处理ROS回调
        ros::Duration(0.1).sleep(); // 控制循环频率
    }

    // 停止机器人
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);

    ROS_INFO("Robot moved %.2f meters at 0.1 m/s", linear_x);
}


// Function to rotate the robot locally
void rotateRobot(double angular_speed, double duration, geometry_msgs::Twist &vel_msg, ros::Publisher &vel_pub) {

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

void wallFollowing(WallSide wall_side, DistancesStruct distances, bool curr_turn, bool prev_turn, float left_dist, float right_dist, float front_dist, float target_distance, float min_speed, float k, float alpha, geometry_msgs::Twist &vel, ros::Publisher &vel_pub) {
    float safe_threshold = 0.5;

    float max_speed = 0.25;

    // Update linear speed based on wall distances
    if (distances.min >= safe_threshold) {
        vel.linear.x = max_speed;
    }
    else {
        vel.linear.x = min_speed + (max_speed - min_speed) * ((distances.min - target_distance) / (safe_threshold - target_distance));
        vel.linear.x = std::max(static_cast<double>(min_speed), std::min(static_cast<double>(max_speed), vel.linear.x));
    }
    
    bumper_handling(vel, vel_pub);
    // **Wall-Following Logic**
    if (front_dist > 0.9) {
        if (wall_side == LEFT) {
            // Follow the left wall
            if (left_dist < target_distance) {
                vel.angular.z = -k * (1 - exp(-alpha * left_dist)); // Adjust right
            } else if (left_dist > target_distance) {
                
                vel.angular.z = Deg2Rad(28);  // Adjust left
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

    else if (front_dist < 0.68 & left_dist < 0.6 & right_dist < 0.68 ) {
        // vel.angular.z = -1.57;  // 1.57 radians = 90 degrees
        ROS_INFO("surrounded by three walls");
        rotateRobot(-0.25, 12.56, vel, vel_pub);
        rotateRobot(0, 0, vel, vel_pub);
        curr_turn = true;
        
    }
    else if (front_dist < 0.9 & left_dist < 0.9 & right_dist > 0.9  & curr_turn != prev_turn) {
        // vel.angular.z = -1.57;  // 1.57 radians = 90 degrees
        ROS_INFO("Turn to the right");
        rotateRobot(-0.18, 6.28, vel, vel_pub);
        rotateRobot(0, 0, vel, vel_pub);
        curr_turn = true;
    }
    else if (front_dist < 0.9 & left_dist > 0.9 & right_dist < 0.9 & curr_turn != prev_turn) {
        // vel.angular.z = -1.57;  // 1.57 radians = 90 degrees
        ROS_INFO("Turn to the left");
        rotateRobot(0.18, 6.28, vel, vel_pub);
        rotateRobot(0, 0, vel, vel_pub);
        curr_turn = true;
    }
    else {
        // Obstacle detected in front, slow down and turn
        vel.linear.x = min_speed; // Slow down
        vel.angular.z = (wall_side == LEFT) ? -0.26 : 0.26; // Turn away from the wall
    }
}

void bumper_handling (geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    if(bumpers.anyPressed){
        if(bumpers.leftPressed){
            handleBumperPressed2((float) -45.0, vel, vel_pub);
        }

        else if(bumpers.rightPressed){
            handleBumperPressed2((float) 45.0, vel, vel_pub);
        }

        else if(bumpers.centerPressed){
            handleBumperPressed2((float) 0.0, vel, vel_pub);
        }
        }
}

void handleBumperPressed2(float turnAngle, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("handleBumperPressed2() called...");
    float reverseDistance = 0.2;
    float forwardDistance = reverseDistance / std::cos(Deg2Rad(turnAngle)) * 0.9;

    float exitDistanceThreshold = 0.02;

    // 1. Reverse
    ROS_INFO("handleBumperPressed2() | Reversing...");
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
    ROS_INFO("handleBumperPressed2() | Turning...");
    rotateToHeading(yaw + turnAngle, vel, vel_pub);

    // 3. Drive Forward
    ROS_INFO("handleBumperPressed2() | Advancing...");
    x0 = posX;
    y0 = posY;
    dx = 0;
    dy = 0;
    d = 0;
    ROS_INFO("%.2f", d-forwardDistance);
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
    ROS_INFO("handleBumperPressed2() | Correcting yaw...");
    rotateToHeading(yaw - turnAngle * 0.7, vel, vel_pub);



    ROS_INFO("handleBumperPressed2() | END");
    linear = 0;
    angular = 0;

    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    return;

}

