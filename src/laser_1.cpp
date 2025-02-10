#include "laser_1.h"


void orthogonalizeRay(int ind, int nLasers, float distance, float &horz_dist, float &front_dist) {
    float angle = (float) ind / (float) nLasers * full_angle + 90 - full_angle / 2;
    horz_dist = distance * std::cos(DEG2RAD(angle));
    front_dist = distance * std::sin(DEG2RAD(angle));
    front_dist += 0.05;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    nLasers = static_cast<int>((msg->angle_max - msg->angle_min) / msg->angle_increment);
    // 更新全局变量的值
    right_idx = 0;
    front_idx = nLasers / 2;
    left_idx = nLasers - 1;


    laser_data.left_distance = msg->ranges[left_idx];
    laser_data.front_distance = msg->ranges[front_idx];
    laser_data.right_distance = msg->ranges[right_idx];

    // int i = left_idx-1;
    while(std::isnan(laser_data.left_distance)){
        laser_data.left_distance = msg->ranges[left_idx];
        left_idx--;
    }

    // i = right_idx+1;
    while(std::isnan(laser_data.right_distance)){
        laser_data.right_distance = msg->ranges[right_idx];
        right_idx++;
    }


    int i = 1;
    bool odd = true;

    while(std::isnan(laser_data.front_distance)){
        laser_data.front_distance = msg->ranges[front_idx+i];
        if(odd){
            odd = false;
            i = -(i);
        }
        else{
            odd = true;
            i = -(i+1);
        }
    }

    front_idx += i;

    // Helper function to compute the average of three rays, handling NaNs
    auto avg_range = [&](int idx) -> float {
        float sum = 0.0;
        int count = 0;

        if (idx > 0 && !std::isnan(msg->ranges[idx - 1])) {
            sum += msg->ranges[idx - 1];
            count++;
        }
        if (!std::isnan(msg->ranges[idx])) {
            sum += msg->ranges[idx];
            count++;
        }
        if (idx < nLasers - 1 && !std::isnan(msg->ranges[idx + 1])) {
            sum += msg->ranges[idx + 1];
            count++;
        }

        return (count > 0) ? (sum / count) : std::numeric_limits<float>::quiet_NaN();
    };

    // Store processed values in the struct
    // laser_data.left_distance = avg_range(left_idx);
    // laser_data.front_distance = avg_range(front_idx);
    // laser_data.right_distance = avg_range(right_idx);

    // Compute min distance while ignoring NaNs
    laser_data.min_distance = std::numeric_limits<float>::infinity();
    for (int i = 0; i < nLasers; ++i) {
        if (!std::isnan(msg->ranges[i])) {
            laser_data.min_distance = std::min(laser_data.min_distance, msg->ranges[i]);
        }
    }

    ROS_INFO("Left: %.2f m, Front: %.2f m, Right: %.2f m, Min: %.2f m",
             laser_data.left_distance, laser_data.front_distance,
             laser_data.right_distance, laser_data.min_distance);
}
