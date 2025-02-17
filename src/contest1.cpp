//PERCY
#include "bumper.h"
#include "laser.h"
#include "movement.h"
#include "biasedExplore.h"
#include "wallFollowing.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>


// Define global publishers declared as extern in bumper.h
ros::Publisher pose_pub;
ros::Publisher marker_pub;


enum Mode {WALL_FOLLOW, RANDOM_NAVIGATE};


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;


    // Subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber subOdom = nh.subscribe("odom", 1, &odomCallback);


    // Publishers
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("bumper_pose", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


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
    const double k = 0.17;   // Scaling factor for angular velocity
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


    Mode mode = WALL_FOLLOW;
    bool fullRoundCompleted = false;

    //initial state check
    sweptPoints.clear();
    sweep360(sweptPoints, vel, vel_pub);
    // find left wall
    std::array<float, 2> leftWall = findLeftWall(sweptPoints);
    if (leftWall[0] != -1 && leftWall[1] != -1) {
        ROS_INFO("Left Wall found at: (%.2f, %.2f)", leftWall[0], leftWall[1]);
    } else {
        ROS_WARN("No left wall detected!");
    }

    findFirstDestination(posX, posY, sweptPoints, visitedPoints, nextX, nextY);
    rotateToStarting(nextX, nextY, vel, vel_pub);
 


    while(ros::ok()) {
        ros::spinOnce();


        switch (mode) {
            case WALL_FOLLOW: {

                get_coord();
                corners = filter_corner();

                // Check distances
                front_dist = std::isnan(distances.frontRay) ? safe_threshold : distances.frontRay;
                left_dist = std::isnan(distances.leftRay) ? safe_threshold : distances.leftRay;
                right_dist = std::isnan(distances.rightRay) ? safe_threshold : distances.rightRay;

                curr_turn = false;

                // Determine if wall is being followed based on left and right distances
                left_change = left_dist - prev_left_distance;
                right_change = right_dist - prev_right_distance;
                if (left_change < 0.1 || right_change < 0.1){
                    wall_following = true;
                }

                // Main Wall Following Algorithm
                if (left_change > corridor_threshold && wall_following) {
                
                    ROS_WARN("Detected corridor on the LEFT!");
                   
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


                else if (right_change > corridor_threshold && wall_following) {

                    ROS_WARN("Detected corridor on the RIGHT!");
                    vel.angular.z = 0.0;  // No adjustment needed

                    // Initialize current position if this is the first corridor detection
                    if (corridor_count < 1) {
                        current_x = posX;
                        current_y = posY;
                        corridor_count += 1;
                    }


                    moveRobot(distances.rightVertPrev, 0, vel, vel_pub);
                    ROS_WARN("front distance move %.1f°", distances.rightVertPrev);
                    wall_following = false;
                }


                else {
                
                    WallSide wall_side = LEFT; // Change to RIGHT to follow the right wall
                  
                    wallFollowing(wall_side, distances, curr_turn, prev_turn, left_dist, right_dist, front_dist, target_distance, min_speed, k, alpha, vel, vel_pub);
                }


                prev_left_distance = left_dist;
                prev_right_distance = right_dist;
                prev_turn = curr_turn;


                vel_pub.publish(vel);


                // Loop Checker
                if (is_position_visited(posX, posY)) {
                    ROS_INFO("Robot has completed a round and returned to previous position.");
                    fullRoundCompleted = true;
                    mode = RANDOM_NAVIGATE;  // Transition to random navigation
                    break;  // Stop wall-following
                }


                break;
            }
            case RANDOM_NAVIGATE: {
                sweptPoints.clear();
                sweep360(sweptPoints, vel, vel_pub);
                ROS_INFO("Size: %zu", sweptPoints.size());
               
                findNextDestination(posX, posY, sweptPoints, visitedPoints, nextX, nextY);


                ROS_INFO("----------------Visited Positions----------------");
                for(int i = 0; i < visitedPoints.size(); i++){
                    ROS_INFO("(%.2f,%.2f)", visitedPoints[i][0], visitedPoints[i][1]);
                }


                navigateToPosition(nextX, nextY, vel, vel_pub);  
               
                break;
            }
        }

        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }


return 0;
}






