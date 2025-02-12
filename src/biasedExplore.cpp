#include "biasedExplore.h"

float sweepAngular = Deg2Rad(30.0);
float sweepReturnAngularTolerance = 1.5;
int minSweepPoints = 180;
float stopBeforeWallDistance = 0.4;
float distanceLimit = 4;

bool isWallSegment(const std::vector<std::array<float, 2>> &points, int startIdx, int endIdx) {
    int N = endIdx - startIdx + 1;
    if (N < 3) return false;  // 至少需要3个点拟合直线

    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (int i = startIdx; i <= endIdx; i++) {
        double x = points[i][0];  // X 坐标
        double y = points[i][1];  // Y 坐标
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    // 计算直线斜率 k 和误差
    double k = (N * sum_xy - sum_x * sum_y) / (N * sum_x2 - sum_x * sum_x);
    double b = (sum_y - k * sum_x) / N;

    double error = 0;
    for (int i = startIdx; i <= endIdx; i++) {
        double predicted_y = k * points[i][0] + b;
        error += fabs(predicted_y - points[i][1]);
    }
    error /= N;

    return (error < 0.05);  // 误差小于 0.05 认为是墙
}

std::array<float, 2> findLeftWall(const std::vector<std::array<float, 2>> &sweptPoints) {
    for (int i = 5; i < sweptPoints.size() - 5; i++) {  
        if (isWallSegment(sweptPoints, i - 5, i + 5)) {  
            return sweptPoints[i];  // 找到左墙位置
        }
    }
    return {-1, -1};  // 没找到墙
}

void findFirstDestination(float posX, float posY, std::vector<std::array<float, 2>> sweptPoints, 
                         std::vector<std::array<float, 2>> &visitedPoints, float &nextX, float &nextY) {
    int selectedIndex = -1;
    float maxSum = -1e9;  // negative infinity to ensure it can be updated
    float thisSum;

    float weightedK = 0.1;  // exponential weighting coefficient
    float jCoefficient;

    // Find the left wall position
    std::array<float, 2> leftWall = findLeftWall(sweptPoints);
    ROS_INFO("Left wall position: %.2f, %.2f", leftWall[0], leftWall[1]);

    // If no wall is detected, just return without finding a destination
    if (leftWall[0] == -1 && leftWall[1] == -1) {
        ROS_WARN("No left wall detected. Cannot find destination.");
        return;
    }

    // Record current position
    std::array<float, 2> currentPosition = {posX, posY};
    visitedPoints.push_back(currentPosition);

    // Iterate over all swept points and find the best target point
    for (int i = 0; i < sweptPoints.size(); i++) {
        thisSum = 0;

        // Calculate weighted distance to all visited points
        for (int j = 0; j < visitedPoints.size(); j++) {
            jCoefficient = exp(weightedK * j);  // higher weight for earlier visited points
            thisSum += jCoefficient * pow(distanceBetween(visitedPoints[j][0], visitedPoints[j][1], 
                                                          sweptPoints[i][0], sweptPoints[i][1]), 0.5);
        }

        // Choose the point with the highest weighted distance sum
        if (thisSum > maxSum) {
            maxSum = thisSum;
            selectedIndex = i;
        }
    }

    // Select the best destination point
    if (selectedIndex != -1) {
        nextX = sweptPoints[selectedIndex][0];
        nextY = sweptPoints[selectedIndex][1];
        ROS_INFO("NextX/NextY before: %.2f/%.2f", nextX, nextY);

        // Calculate distance to target and stop before the wall
        float nextDist = distanceBetween(posX, posY, nextX, nextY) - stopBeforeWallDistance;
        
        // Calculate target orientation angle
        float nextYaw = Rad2Deg(atan2(nextY - posY, nextX - posX));
        
        // Adjust target angle so that the robot's left side faces the wall
        nextYaw += 90.0f;

        // Recalculate target position based on adjusted yaw angle
        nextX = posX + nextDist * cos(Deg2Rad(nextYaw));
        nextY = posY + nextDist * sin(Deg2Rad(nextYaw));

        ROS_INFO("NextX/NextY after: %.2f/%.2f", nextX, nextY);
        ROS_INFO("Distance of %.2f at index %d.", maxSum, selectedIndex);
    } else {
        ROS_WARN("No valid next destination found!");
    }
}

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

    float weightedK = 0.1;
    float jCoefficient;

    std::array<float,2> currentPosition = {posX, posY};
    visitedPoints.push_back(currentPosition);


    for(int i = 0; i < sweptPoints.size(); i++){
        thisSum = 0;

        for(int j = 0; j < visitedPoints.size(); j++){
            jCoefficient = exp(weightedK * j);
            thisSum += jCoefficient * (float) pow(distanceBetween(visitedPoints[j][0], visitedPoints[j][1], sweptPoints[i][0], sweptPoints[i][1]), 0.5);
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

    float nextDist = distanceBetween(posX, posY, nextX, nextY) - stopBeforeWallDistance;
    float nextYaw = Rad2Deg(atan2(nextY - posY, nextX - posX));
    
    // ROS_INFO("Distance of %.2f at index %d.", nextDist, selectedIndex);
    nextX = posX + nextDist * std::cos(Deg2Rad(nextYaw));
    nextY = posY + nextDist * std::sin(Deg2Rad(nextYaw));
    
    ROS_INFO("NextX/NextY after: %.2f/%.2f", nextX, nextY);

    ROS_INFO("Distance of %.2f at index %d.", maxSum, selectedIndex);

}