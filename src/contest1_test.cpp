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
#include <vector>  // Include for vector usage

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

double posX = 0., posY = 0., yaw = 0.;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
float angular = 0.0;
float linear = 0.0;
float minLaserDist = std::numeric_limits<float>::infinity();

float left_distance = 0.0, right_distance = 0.0, front_distance = 0.0;
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5;

// Create a vector to store positions
std::vector<std::pair<double, double>> positions;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
   bumper[msg->bumper] = msg->state;
}

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
   posX = msg->pose.pose.position.x;
   posY = msg->pose.pose.position.y;
   yaw = tf::getYaw(msg->pose.pose.orientation);
   ROS_INFO("(x,y):(%f,%f) Orint: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   minLaserDist = std::numeric_limits<float>::infinity();
   nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;

   int right_idx = 0;                 
   int front_idx = nLasers / 2;      
   int left_idx = nLasers - 1;      

   right_distance = msg->ranges[right_idx];
   front_distance = msg->ranges[front_idx];
   left_distance = msg->ranges[left_idx];

   ROS_INFO("Left (first) distance: %.2f m", left_distance);
   ROS_INFO("Front (middle) distance: %.2f m", front_distance);
   ROS_INFO("Right (last) distance: %.2f m", right_distance);
  
   if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
       for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
           minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
       }
   }
   else {
       for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
       }
   }
}

void get_coord() {
   // Store the current coordinates in the vector if they are unique
   bool is_visited = false;
   for (const auto& coord : positions) {
       if (std::abs(coord.first - posX) < 0.05 && std::abs(coord.second - posY) < 0.05) {
           is_visited = true;
           break;
       }
   }

   if (!is_visited) {
       positions.push_back(std::make_pair(posX, posY)); // Only store new coordinates
       ROS_INFO("Stored coordinates: (%f, %f)", posX, posY);
   } else {
       ROS_INFO("Coordinates (%f, %f) already visited, not storing again.", posX, posY);
   }
}

// Function to check if the robot has returned to a previously visited position
bool is_position_visited(double x, double y, double threshold = 0.05) {
   for (const auto& coord : positions) {
       if (std::abs(coord.first - x) < threshold && std::abs(coord.second - y) < threshold) {
           return true; // Position has been visited
       }
   }
   return false; // Position has not been visited
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

       // Call get_coord() to store positions during the loop
       get_coord();

       // Check if the robot has returned to a previous position
       if (is_position_visited(posX, posY)) {
           ROS_INFO("Robot has returned to a previously visited position.");
           break;  // Stop the loop if the robot returns to a visited position
       }

       bool any_bumper_pressed = false;
       float target_distance = 0.9;
       if (front_distance > 1.0 && !std::isnan(front_distance) && !std::isnan(left_distance) && !std::isnan(right_distance)) {
  
           const double k = 0.15;   
           const double alpha = 1.5;

           if (left_distance < target_distance) {
               angular = -k * (1 - exp(-alpha * left_distance)); 
               linear = 0.1;   
           }
           else if (left_distance > target_distance) {
               angular = k * (1 - exp(-alpha * left_distance));  
               linear = 0.1;   
           } 
           else {
               angular = 0.0;                                  
               linear = 0.1;                                   
           }
       }
       else {
           linear = 0.04;
           angular = -0.26;                    
       }

       vel.angular.z = angular;
       vel.linear.x = linear;
       vel_pub.publish(vel);

       secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
       loop_rate.sleep();
   }

   // After the loop ends, print all the stored coordinates
   ROS_INFO("Coordinates visited:");
   for (const auto& coord : positions) {
       ROS_INFO("X: %.2f, Y: %.2f", coord.first, coord.second);
   }

   return 0;
}


