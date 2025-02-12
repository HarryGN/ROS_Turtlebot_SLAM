#include "bumper.h"

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
BumpersStruct bumpers;

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

