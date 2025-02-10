//PERCY
#include "bumper.h"
#include "laser.h"
#include "movement.h"
#include "biasedExplore.h"
#include "wallFollowing.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber subOdom = nh.subscribe("odom", 1, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    angular = 0.0;
    linear = 0.0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);

    std::vector<std::array<float, 2>> sweptPoints;
    std::vector<std::array<float, 2>> visitedPoints;
    float nextX, nextY;


    #pragma region wallFollowing Param
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
    bool prev_turn = false;
    bool curr_turn = false;

    float prev_left_distance = 0.0, prev_right_distance = 0.0;

    float front_dist;
    float left_dist;
    float right_dist;

    float corridor_threshold = 0.6;   // 设定通道触发阈值
    float max_distance_change = 1.5;  // 设定最大突变距离
    float min_rotation = 30.0;        // 最小旋转角度
    float max_rotation = 90.0;        // 最大旋转角度
    float left_change;
    float right_change;

    std::pair<std::pair<double, double>, std::pair<double, double>> corners;

    #pragma endregion
    // wallFollowing
    bool wallFollowingLoop = true;
    while(ros::ok() && secondsElapsed <= 300 && wallFollowingLoop) {
        ros::spinOnce();

        get_coord();

        corners = filter_corner();
        // Print corner coord
        // ROS_INFO("Corner 1: (%.2f, %.2f)", corners.first.first, corners.first.second); 
        // ROS_INFO("Corner 2: (%.2f, %.2f)", corners.second.first, corners.second.second);


        // Check distances
        front_dist = std::isnan(distances.frontRay) ? safe_threshold : distances.frontRay;
        left_dist = std::isnan(distances.leftRay) ? safe_threshold : distances.leftRay;
        right_dist = std::isnan(distances.rightRay) ? safe_threshold : distances.rightRay;

        // ???
        curr_turn = false;

        // Update linear speed based on wall distances
        if (distances.min >= safe_threshold) {
            vel.linear.x = max_speed;
        }
        else {
            vel.linear.x = min_speed + (max_speed - min_speed) * ((distances.min - target_distance) / (safe_threshold - target_distance));
            vel.linear.x = std::max(static_cast<double>(min_speed), std::min(static_cast<double>(max_speed), vel.linear.x));
        }

        // Determine if wall is being followed based on left and right distances
        left_change = left_dist - prev_left_distance;
        right_change = right_dist - prev_right_distance;
        if (left_change < 0.1 || right_change < 0.1){
            wall_following = true;
        }


        // Main Wall Following Algorithm
        if (left_change > corridor_threshold & wall_following) {
            // Calculate rotation angle (clamped between min_rotation and max_rotation)
            float rotation_angle_deg = min_rotation + (left_change - corridor_threshold) * (max_rotation - min_rotation) / (max_distance_change - corridor_threshold);
            rotation_angle_deg = std::min(max_rotation, std::max(min_rotation, rotation_angle_deg)); // Clamp the angle
            float rotation_angle_rad = Deg2Rad(rotation_angle_deg); // Convert to radians

            ROS_WARN("Detected corridor on the LEFT! Turning %.1f°", rotation_angle_deg);
            
            vel.angular.z = 0.0;  // No adjustment needed

            // Initialize current position if this is the first corridor detection
            if (corridor_count < 1) {
                current_x = posX;
                current_y = posY;
                corridor_count += 1;
            }

            moveRobot(distances.leftVertPrev, 0, vel, vel_pub);
            ROS_WARN("front distance move %.1f°", distances.leftVertPrev);
            wall_following = false;
        }

        else if (right_change > corridor_threshold & wall_following) {
            // 计算旋转角度 (限制 30° ~ 90°)
            float rotation_angle_deg = min_rotation + (right_change - corridor_threshold) * (max_rotation - min_rotation) / (max_distance_change - corridor_threshold);
            rotation_angle_deg = std::min(max_rotation, std::max(min_rotation, rotation_angle_deg));
            float rotation_angle_rad = Deg2Rad(rotation_angle_deg);

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

        prev_left_distance = left_dist;
        prev_right_distance = right_dist;
        prev_turn = curr_turn;

        vel_pub.publish(vel);


        // Loop Checker
        if (is_position_visited(posX, posY)) {
            ROS_INFO("Robot has completed a round and returned to previous position.");
            
            //get_all_corners
            std::vector<std::pair<double, double>> corners = get_all_corners();
            
            // Printing out the corners for debugging or monitoring
            ROS_INFO("Detected Corners:");
            for (const auto& corner : corners) {
                ROS_INFO("Corner: (X: %.2f, Y: %.2f)", corner.first, corner.second);
            }

            bool wallFollowingLoop = false;
	        break;  // Stop the loop if visited position
        }




        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }



    // biasedExplore
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        sweptPoints.clear();
        sweep360(sweptPoints, vel, vel_pub);
        ROS_INFO("Size: %zu", sweptPoints.size());
        
        findNextDestination(posX, posY, sweptPoints, visitedPoints, nextX, nextY);

        ROS_INFO("----------------Visited Positions----------------");
        for(int i = 0; i < visitedPoints.size(); i++){
            ROS_INFO("(%.2f,%.2f)", visitedPoints[i][0], visitedPoints[i][1]);
        }

        navigateToPosition(nextX, nextY, vel, vel_pub);      

        // vel.angular.z = angular;
        // vel.linear.x = linear;
        // vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}