//PERCY
#include "bumper.h"
#include "laser.h"
#include "movement.h"
#include "biasedExplore.h"

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

    // wallFollowing
    while(ros::ok() && secondsElapsed <= 180) {
        ros::spinOnce();

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