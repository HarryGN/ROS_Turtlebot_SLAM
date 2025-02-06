#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cstdlib>
#include <ctime>

#define RAD2DEG(rad) ((rad) * 180. / M_PI)

double posX = 0., posY = 0., yaw = 0.;
float minLaserDist = std::numeric_limits<float>::infinity();
float safeDistance = 0.5;  // Safe distance from any obstacle

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    minLaserDist = std::numeric_limits<float>::infinity();
    for (uint32_t i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < minLaserDist) {
            minLaserDist = msg->ranges[i];
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_explorer");
    ros::NodeHandle nh;

    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odomCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Rate loop_rate(10);

    srand(time(NULL));  // Seed the random number generator

    while (ros::ok()) {
        ros::spinOnce();

        geometry_msgs::Twist vel;
        if (minLaserDist > safeDistance) {
            // Randomly choose to go forward or turn
            int action = rand() % 3;  // 0 = forward, 1 = turn left, 2 = turn right

            switch (action) {
                case 0:
                    vel.linear.x = 0.2;
                    vel.angular.z = 0;
                    break;
                case 1:
                    vel.linear.x = 0;
                    vel.angular.z = 0.5;
                    break;
                case 2:
                    vel.linear.x = 0;
                    vel.angular.z = -0.5;
                    break;
            }
        } else {
            // Turn in place if too close to an obstacle
            vel.linear.x = 0;
            vel.angular.z = 0.5;  // Adjust this value to control the turn rate
        }
        
        vel_pub.publish(vel);
        loop_rate.sleep();
    }

    return 0;
}



