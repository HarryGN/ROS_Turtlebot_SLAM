#ifndef updateCoordinateHeader
#define updateCoordinateHeader 

#include <vector>
#include <utility>
#include <cmath>
#include <ros/ros.h>

// Forward declaration of any external variables used
extern float posX, posY; // Assuming these are declared globally elsewhere

void get_coord();
double calculate_distance(const std::pair<double, double>& p1, const std::pair<double, double>& p2);
std::pair<std::pair<double, double>, std::pair<double, double>> filter_corner();
double get_total_dist();
bool is_position_visited(double x, double y, double threshold = 1.0, double min_distance = 4);
double point_to_line_distance(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end);
int line_side(const std::pair<double, double>& point, const std::pair<double, double>& line_start, const std::pair<double, double>& line_end);
std::vector<std::pair<double, double>> get_all_corners();

extern std::vector<std::pair<double, double>> positions;


#endif