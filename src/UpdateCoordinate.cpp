#include "UpdateCoordinate.h"

void UpdateCoordinate::get_coord() {
    bool is_visited = false;
    for (const auto& coord : positions) {
        if (std::abs(coord.first - posX) < 0.05 && std::abs(coord.second - posY) < 0.05) {
            is_visited = true;
            break;
        }
    }

    if (!is_visited) {
        positions.push_back(std::make_pair(posX, posY));
        ROS_INFO("Stored coordinates: (%f, %f)", posX, posY);
    } else {
        ROS_INFO("Coordinates (%f, %f) already visited, not storing again.", posX, posY);
    }
}

double UpdateCoordinate::calculate_distance(const std::pair<double, double>& p1, const std::pair<double, double>& p2) {
    return std::sqrt(std::pow(p2.first - p1.first, 2) + std::pow(p2.second - p1.second, 2));
}

std::pair<std::pair<double, double>, std::pair<double, double>> UpdateCoordinate::filter_corner() {
    double max_distance = 0.0;
    std::pair<double, double> corner1, corner2;

    for (size_t i = 0; i < positions.size(); ++i) {
        for (size_t j = i + 1; j < positions.size(); ++j) {
            double distance = calculate_distance(positions[i], positions[j]);

            if (distance > max_distance) {
                max_distance = distance;
                corner1 = positions[i];
                corner2 = positions[j];
            }
        }
    }

    return std::make_pair(corner1, corner2);
}

double UpdateCoordinate::get_total_dist() {
    double total_dist = 0.0;

    for (size_t i = 1; i < positions.size(); ++i) {
        total_dist += calculate_distance(positions[i - 1], positions[i]);
    }

    return total_dist;
}

bool UpdateCoordinate::is_position_visited(double x, double y, double threshold, double min_distance) {
    double total_dist = get_total_dist();
    ROS_INFO("Total distance = %f", total_dist);
    bool corner_set = false;

    if (total_dist > min_distance) {
        corner_set = true;
        ROS_INFO("Corner_set Flag = TRUE");

        std::pair<std::pair<double, double>, std::pair<double, double>> corners = filter_corner();

        if (std::abs(corners.first.first - x) < threshold && std::abs(corners.first.second - y) < threshold) {
            return true;
        }
    }
    return false;
}

double UpdateCoordinate::point_to_line_distance(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end) {
    double normal_length = std::hypot(line_end.first - line_start.first, line_end.second - line_start.second);
    return std::abs((point.first - line_start.first) * (line_end.second - line_start.second) - (point.second - line_start.second) * (line_end.first - line_start.first)) / normal_length;
}

int UpdateCoordinate::line_side(const std::pair<double, double>& point, the std::pair<double, double>& line_start, the std::pair<double, double>& line_end) {
    double result = (point.first - line_start.first) * (line_end.second - line_start.second) - (point.second - line_start.second) * (line_end.first - line_start.first);
    return (result > 0) ? 1 : (result < 0) ? -1 : 0;
}

std::vector<std::pair<double, double>> UpdateCoordinate::get_all_corners() {
    std::vector<std::pair<double, double>> all_corners;
    std::pair<std::pair<double, double>, std::pair<double, double>> far_corners = filter_corner();

    all_corners.push_back(far_corners.first);
    all_corners.push_back(far_corners.second);

    double max_distance_1 = 0.0, max_distance_2 = 0.0;
    std::pair<double, double> corner_3, corner_4;
    int side_first = line_side(positions.front(), far_corners.first, far_corners.second);

    for (const auto& coord : positions) {
        double dist = point_to_line_distance(coord, far_corners.first, far_corners.second);
        int side = line_side(coord, far_corners.first, far_corners.second);

        if (side != side_first && dist > max_distance_1) {
            max_distance_1 = dist;
            corner_3 = coord;
        } else if (side == side_first && dist > max_distance_2) {
            max_distance_2 = dist;
            corner_4 = coord;
        }
    }

    all_corners.push_back(corner_3);
    all_corners.push_back(corner_4);

    return all_corners;
}
