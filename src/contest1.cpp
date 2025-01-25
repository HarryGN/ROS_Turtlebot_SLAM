//PERCY
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
#include <thread>

#include <math.h>

float angular;
float linear;
float posX = 0.0, posY = 0.0, yaw = 0.0;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool leftBumperPressed;
bool centerBumperPressed;
bool rightBumperPressed;

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

float Deg2Rad(float degrees) {
    return degrees * (M_PI / 180);
}


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    bumper[msg->bumper] = msg->state;
    leftBumperPressed = bumper[kobuki_msgs::BumperEvent::LEFT];
    centerBumperPressed = bumper[kobuki_msgs::BumperEvent::CENTER];
    rightBumperPressed = bumper[kobuki_msgs::BumperEvent::RIGHT];
    ROS_INFO("BUMPER STATES L/C/R: %u/%u/%u", leftBumperPressed, centerBumperPressed, rightBumperPressed);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber subOdom = nh.subscribe("odom", 1, &callbackOdom);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Bumper Event Settings
    bool bumperStepBack = false;
    uint64_t tBumperEventStart;
    uint64_t dBumperStepBack = 2;

    uint64_t dBumperEvent[4] = {3,7,3,7};
    uint64_t dBumperEventTotal = 0;
    for(int i = 0; i < sizeof(dBumperEvent)/sizeof(*dBumperEvent); i++){
        dBumperEventTotal += dBumperEvent[i];
    }


    angular = 0.0;
    linear = 0.1;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        

        // Bumper Event
        if(leftBumperPressed || centerBumperPressed || rightBumperPressed){
            bumperStepBack = true;
            tBumperEventStart = secondsElapsed;
        }

        if(bumperStepBack){
            uint64_t dBumperEventRemaining = secondsElapsed - tBumperEventStart;

            if(dBumperEventRemaining < dBumperEvent[0]){
                linear = -0.1;
                angular = 0.0;
            }
            
            else if(dBumperEventRemaining < dBumperEvent[0] + dBumperEvent[1]){
                linear = 0.0;
                angular = Deg2Rad(15);
            }

            else if(dBumperEventRemaining < dBumperEvent[0] + dBumperEvent[1] + dBumperEvent[2]){
                linear = 0.1;
                angular = 0.0;
            } 
            
            else if (dBumperEventRemaining < dBumperEvent[0] + dBumperEvent[1] + dBumperEvent[2] + dBumperEvent[3]){
                linear = 0.0;
                angular = Deg2Rad(-15);
            }

            else{
                bumperStepBack = false;
                angular = 0.0;
                linear = 0.1;
            }
        }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}