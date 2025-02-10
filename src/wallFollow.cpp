#include "wallFollow.h"


// Function to move the robot with a given linear and angular velocity
void moveRobot(double linear_x, double angular_z, ros::Publisher &vel_pub) {
    geometry_msgs::Twist vel_msg;

    // Set the linear and angular velocity
    vel_msg.linear.x = linear_x;
    vel_msg.angular.z = angular_z;

    // Publish the velocity message
    vel_pub.publish(vel_msg);
    ROS_INFO("Publishing velocity command: linear_x = %f, angular_z = %f", linear_x, angular_z);
}

// Function to rotate the robot locally
void rotateRobot(double angular_speed, double duration, ros::Publisher &vel_pub) {
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
        rotateRobot(-0.25, 12.56, vel_pub);
        rotateRobot(0, 0, vel_pub);
        curr_turn = true;
        
    }
    else if (front_dist < 0.9 & left_dist < 0.9 & right_dist > 0.9  & curr_turn != prev_turn) {
        // vel.angular.z = -1.57;  // 1.57 radians = 90 degrees
        ROS_INFO("Turn to the right");
        rotateRobot(-0.25, 6.28, vel_pub);
        rotateRobot(0, 0, vel_pub);
        curr_turn = true;
    }
    else if (front_dist < 0.9 & left_dist > 0.9 & right_dist < 0.9 & curr_turn != prev_turn) {
        // vel.angular.z = -1.57;  // 1.57 radians = 90 degrees
        ROS_INFO("Turn to the left");
        rotateRobot(0.25, 6.28, vel_pub);
        rotateRobot(0, 0, vel_pub);
        curr_turn = true;
    }
    else {
        // Obstacle detected in front, slow down and turn
        vel.linear.x = min_speed; // Slow down
        vel.angular.z = (wall_side == LEFT) ? -0.26 : 0.26; // Turn away from the wall
    }
}
