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

#define N_BUMPER (3)
#define Rad2Deg(rad) ((rad) * 180. / M_PI)
#define Deg2Rad(deg) ((deg) * M_PI / 180.)

float angular;
float linear;
float posX = 0.0, posY = 0.0, yaw = 0.0;
float rotationTolerance = Deg2Rad(1);
float kp_r = 1;
float kn_r = 0.7;
float minAngular = 10; // Degrees per second
float maxAngular = 90; // Degrees per second

float navigationTolerance = 0.2;
float kp_n = 0.02;
float kn_n = 0.5;
float minLinear = 0.1;
float maxLinear = 0.45;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool leftBumperPressed;
bool centerBumperPressed;
bool rightBumperPressed;

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
    yaw = Rad2Deg(tf::getYaw(msg->pose.pose.orientation));
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, Rad2Deg(yaw));

    if(!posX==posX || !posY==posY || !yaw==yaw ){
        ROS_INFO("NAN ERROR");
        while(true){
            continue;
        }
    }
}

float absPow(float base, float exp){
    if(base < 0){
        return (float) -1*pow(-1*base, exp);
    }
    else{
        return (float) pow(base, exp);
    } 
}

void applyMagnitudeLimits(float &value, float lowerLimit, float upperLimit){
    if(value < 0){
        if(value < -upperLimit){
            value = -upperLimit;
        }
        else if(value > -lowerLimit){
            value = -lowerLimit;
        }
    }

    else if(value > 0){
        if(value > upperLimit){
            value = upperLimit;
        }
        else if(value < lowerLimit){
            value = lowerLimit;
        }
    }
}

float computeAngular(float targetHeading, float currentYaw){
    float angularDeg;

    // Calculate proportional component and then calculate angularDeg based on if it is negative or positive
    float proportional = kp_r*(targetHeading-currentYaw);
    if(proportional < 0){
        angularDeg = (float) -1*pow(-1*proportional, kn_r);
    }
    else{
        angularDeg = (float) pow(proportional, kn_r);
    }

    applyMagnitudeLimits(angularDeg, minAngular, maxAngular);

    return Deg2Rad(angularDeg);
}

void rotateToHeading(float targetHeading, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("ROTATE TO HEADING CALLED");
    ros::spinOnce();

    float proportional;
    while(abs(targetHeading - yaw) > rotationTolerance){
        // Calculate the angular velocity based on PN control algorithm
        // proportional = kp_r*(targetHeading-yaw);
        // if(proportional < 0){
        //     angular = (float) pow(-1*proportional, kn_r);
        // }
        // else{
        //     angular = (float) pow(proportional, kn_r);
        // }

        // // Adjust angular to not be above or below the threshold
        // if(angular > maxAngular){
        //     angular = maxAngular;
        // }
        // else if(angular < minAngular){
        //     angular = minAngular;
        // }
        angular = computeAngular(targetHeading, yaw);

        ros::spinOnce();
        vel.angular.z = angular;
        vel.linear.x = 0;
        vel_pub.publish(vel);

        ROS_INFO("Target/Current Yaw: %f/%f degs | Setpoint: %f degs/s", targetHeading, yaw, Rad2Deg(angular));
    }

    vel.angular.z = 0;
    vel.linear.x = 0;
    vel_pub.publish(vel);

}


void navigateToPosition(float x, float y, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("navigateToPosition() CALLED");
    ros::spinOnce();
    
    float dx = x-posX;
    float dy = y-posY;
    float d = (float) sqrt(pow(dx, 2) + pow(dy, 2));

    // Set and rotate to initial heading
    float targetHeading = Rad2Deg(atan2(dy, dx));
    rotateToHeading(targetHeading, vel, vel_pub);

    // While loop until robot gets there
    while(d > navigationTolerance){
        ros::spinOnce();
        dx = x-posX;
        dy = y-posY;
        d = (float) sqrt(pow(dx, 2) + pow(dy, 2));

        linear = (float) pow(kp_n*d, kn_n);
        applyMagnitudeLimits(linear, minLinear, maxLinear);

        targetHeading = Rad2Deg(atan2(dy, dx));
        angular = computeAngular(targetHeading, yaw);

        vel.angular.z = 0;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        ROS_INFO("Tgt X/Y: %f/%f | Pos X/Y: %f/%f | Lin/Ang: %f/%f", x, y, posX, posY, linear, Rad2Deg(angular));
    }

    linear = 0;
    angular = 0;
    vel.angular.z = angular;
    vel.linear.x = 0;
    vel_pub.publish(vel);

    ROS_INFO("Successful exit from navigateToPosition.");
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
    linear = 0.0;

    // rotateToHeading(90, vel, vel_pub);
    // rotateEndlessly(vel, vel_pub);
    navigateToPosition(-1.929,1.346, vel, vel_pub);
    navigateToPosition(-1.7069999999999999,-1.0, vel, vel_pub);
    navigateToPosition(-0.8680000000000001,1.4365, vel, vel_pub);
    navigateToPosition(-0.3763333333333332,-1.1406666666666667, vel, vel_pub);
    navigateToPosition(0.19299999999999984,1.5270000000000001, vel, vel_pub);
    navigateToPosition(0.9543333333333335,-1.2813333333333332, vel, vel_pub);
    navigateToPosition(1.2539999999999998,1.6175, vel, vel_pub);
    navigateToPosition(2.285,-1.422, vel, vel_pub);
    navigateToPosition(2.3149999999999995,1.708, vel, vel_pub);

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        #pragma region Bumper
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
        #pragma endregion

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}