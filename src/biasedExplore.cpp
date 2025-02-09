#include "biasedExplore.h"

float sweepAngular = Deg2Rad(45.0);
float sweepReturnAngularTolerance = 1.5;
int minSweepPoints = 180;
float stopBeforeWallDistance = 0.6;
float distanceLimit = 4;

void sweep360(std::vector<std::array<float, 2>> &sweptPoints, geometry_msgs::Twist &vel, ros::Publisher &vel_pub){
    ROS_INFO("sweep360() called...");
    ros::spinOnce();
    angular = sweepAngular;
    linear = 0;
    float startingYaw = yaw;

    // Keep rotating until sweptPoints has at least minSweepPoints numeber of points, then the exit condition is that the original heading is returned to
    float endpointX;
    float endpointY;
    std::array<float, 2> endpoint;

    int lastHeading = std::round(startingYaw) - 1;
    while(sweptPoints.size() < minSweepPoints || std::abs(yaw-startingYaw) > sweepReturnAngularTolerance){
        ros::spinOnce();

        // yaw = 0 until rotated bug workaround
        if(startingYaw == 0){
            startingYaw = yaw;
            continue;
        }

        if(std::round(yaw) != lastHeading){
            endpointX = posX + distances.frontRay * std::cos(Deg2Rad(yaw));
            endpointY = posY + distances.frontRay * std::sin(Deg2Rad(yaw));

            endpoint = {endpointX, endpointY};
        
            sweptPoints.push_back(endpoint);

            lastHeading = std::round(yaw);
        }


        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        
        
    }

    vel.angular.z = (float) 0.0;
    vel.linear.x = (float) 0.0;
    vel_pub.publish(vel);

    ROS_INFO("...sweep360() finished.");
}

void findNextDestination(float posX, float posY, std::vector<std::array<float, 2>> sweptPoints, std::vector<std::array<float, 2>> &visitedPoints, float &nextX, float &nextY){
    int selectedIndex = 0;
    float maxSum = 0;
    float thisSum;

    std::array<float,2> currentPosition = {posX, posY};
    visitedPoints.push_back(currentPosition);


    for(int i = 0; i < sweptPoints.size(); i++){
        thisSum = 0;

        for(int j = 0; j < visitedPoints.size(); j++){
            thisSum += distanceBetween(visitedPoints[j][0], visitedPoints[j][1], sweptPoints[i][0], sweptPoints[i][1]);
        }

        
        
        if(thisSum > maxSum){
            maxSum = thisSum;
            selectedIndex = i;
        }

    }

    // Get nextX and nextY, apply compensation for robot to stop before the wall
    
    
    nextX = sweptPoints[selectedIndex][0];
    nextY = sweptPoints[selectedIndex][1];
    ROS_INFO("NextX/NextY before: %.2f/%.2f", nextX, nextY);

    float nextDist = distanceBetween(posX, posY, nextX, nextY)*0.95 - stopBeforeWallDistance;
    float nextYaw = Rad2Deg(atan2(nextY - posY, nextX - posX));
    
    // ROS_INFO("Distance of %.2f at index %d.", nextDist, selectedIndex);
    nextX = posX + nextDist * std::cos(Deg2Rad(nextYaw));
    nextY = posY + nextDist * std::sin(Deg2Rad(nextYaw));
    
    ROS_INFO("NextX/NextY after: %.2f/%.2f", nextX, nextY);

    ROS_INFO("Distance of %.2f at index %d.", maxSum, selectedIndex);

}