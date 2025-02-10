// Defined
#include "coord.h"
#include "common_1.h"
#include "bumper_1.h"
#include "laser_1.h"
#include "wallFollow.h"


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
