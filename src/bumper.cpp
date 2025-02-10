#include "bumper.h"

// Existing global variables for bumper state
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
BumpersStruct bumpers;

// Make sure to include these headers
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    // Update the global bumper state.
    bumper[msg->bumper] = msg->state;
    bool leftBumperPressed   = bumper[kobuki_msgs::BumperEvent::LEFT];
    bool centerBumperPressed = bumper[kobuki_msgs::BumperEvent::CENTER];
    bool rightBumperPressed  = bumper[kobuki_msgs::BumperEvent::RIGHT];
    ROS_INFO("BUMPER STATES L/C/R: %u/%u/%u", leftBumperPressed, centerBumperPressed, rightBumperPressed);

    // If any bumper is pressed, publish a pose and a marker.
    if (leftBumperPressed || centerBumperPressed || rightBumperPressed) {
        // Publish the current pose.
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map"; // Change to "odom" if needed.
        pose.pose.position.x = posX;
        pose.pose.position.y = posY;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        pose_pub.publish(pose);

        // Create and publish a marker.
        static int marker_id = 0; // Keep track of unique marker IDs.
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map"; // Change to "odom" if needed.
        marker.header.stamp = ros::Time::now();
        marker.ns = "bumper_markers";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = posX;
        marker.pose.position.y = posY;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; // Fully opaque.
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_pub.publish(marker);
    }
}

void handleBumperPressed(float turnAngle, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("handleBumperPressed() called...");
    float reverseDistance = 0.18;
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
            turnAngle = 60;
        }

        else{
            turnAngle = -60;
        }

    }
    ROS_INFO("handleBumperPressed() | Turning...");
    rotateToHeading(yaw + turnAngle, vel, vel_pub);

    // 3. Drive Forward
    if(bumpers.anyPressed){
        return;
    }
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
            handleBumperPressed((float) -60, vel, vel_pub);
        }

        else if(bumpers.rightPressed){
            handleBumperPressed((float) 60, vel, vel_pub);
        }

        else if(bumpers.centerPressed){
            handleBumperPressed((float) 0.0, vel, vel_pub);
        }
    }
}    


void checkBumper(geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    if(bumpers.anyPressed){
        if(bumper[kobuki_msgs::BumperEvent::LEFT]){
            handleBumperPressed(-60.0f, vel, vel_pub);
        }
        else if(bumper[kobuki_msgs::BumperEvent::RIGHT]){
            handleBumperPressed(60.0f, vel, vel_pub);
        }
        else if(bumper[kobuki_msgs::BumperEvent::CENTER]){
            handleBumperPressed(0.0f, vel, vel_pub);
        }
    }
}

