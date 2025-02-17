# ROS TurtleBot SLAM

## Project Overview
This project involves the development of an autonomous navigation and mapping algorithm for the TurtleBot 2 robot using ROS (Robot Operating System). The goal is to explore an unknown environment, map it using the Xbox 360 Kinect sensor, and navigate autonomously within an 8-minute time limit. The robot uses a combination of wall-following and random navigation strategies to maximize coverage and avoid loops.

## Table of Contents
1. [Strategy and Algorithm Used](#strategy-and-algorithm-used)
2. [Detailed Robot Design and Implementation](#detailed-robot-design-and-implementation)
3. [Future Recommendations](#future-recommendations)
4. [References](#references)

---

## Strategy and Algorithm Used
The robot operates in two modes:
1. **Wall Following Mode**: The robot follows the walls of the environment to map the perimeter.
2. **Random Navigation Mode**: After completing a loop around the walls, the robot switches to random navigation to explore the interior.

### Wall Following Mode
- The robot uses laser scan data to maintain a constant distance from the wall.
- It adjusts its heading and reacts to sudden changes, such as detecting corridors.

### Random Navigation Mode
- The robot performs a 360° sensor sweep to gather data on its surroundings.
- It evaluates potential destinations and selects the farthest point from previously visited locations.
- The robot navigates to the selected target while avoiding obstacles.

---

## Detailed Robot Design and Implementation
### Sensory Design
- **Kinect 360 - Laser Sensor**: Detects obstacles and measures distances to walls.
- **Bumper Sensors**: Provide immediate feedback upon collision with obstacles.
- **Odometry Sensors**: Provide real-time position and orientation data.

### Controller Design
- **High-Level Controller**: Determines the navigation mode (Wall Following or Random Navigation).
- **Low-Level Controller**: Computes movement commands based on sensor data.

### Algorithms Developed and Used

#### Wall Following
The **Wall Following** algorithm is the primary mode used by the robot to explore the perimeter of the environment. It is inspired by cleaning robots that follow walls to map a room. The algorithm ensures the robot maintains a constant distance from the wall while navigating around obstacles.

- **Linear Adjustment**: The robot adjusts its angular velocity based on the distance to the wall using an exponential equation:
  \[
  \text{angular} = \pm k \cdot (1 - \exp[-\alpha \cdot l_{\text{left}}])
  \]
  Here, \( k \) is the maximum angular velocity, and \( \alpha \) prevents sudden spikes in velocity due to rapid changes in distance.

- **Turning Algorithm**: The robot detects openings in the wall and executes turning maneuvers. It continuously checks for gaps or openings and adjusts its path accordingly.

- **Opening Detection**: The robot identifies gaps in the wall and turns when an opening is detected. If surrounded by three walls, it performs a 90-degree turn to continue navigation.

- **Closest Wall Finding**: 
  - The robot performs an initial 360° sweep to detect the closest wall using the `findLeftWall()` function.
  - The `isWallSegment()` function applies a least-squares linear regression to fit a line to the scanned points and calculates the deviation of the points from the line. A segment is classified as a wall if the average deviation is below a predefined threshold (0.05).
  - The robot selects the side with the highest concentration of points, as a closer wall produces more intense laser returns. The final output is the closest wall to the robot, which is used to guide the wall-following behavior.

  ![Closest Wall Selection](https://github.com/user-attachments/assets/2afe5d15-2df0-4e27-8497-38354a51035b)

#### Biased Explore
The **Biased Explore** algorithm is used after the robot completes the initial wall-following loop. It ensures the robot explores unexplored areas by selecting targets that are farthest from previously visited locations.

- **360° Sensor Sweep**: The robot performs a full 360° scan to gather data on its surroundings. It stores the endpoint coordinates of each laser scan.
- **Target Selection**: The robot evaluates potential destinations by calculating the weighted distance to previously visited points. The point with the highest weighted distance is selected as the next target.
- **Weighting System**: The algorithm prioritizes points that are farther from visited locations and gives more weight to recently visited points to avoid loops.

#### Movement
The **Movement** algorithm controls the robot's linear and angular velocities using a **PN (Proportional-Nonlinear) control** mechanism. This ensures smooth acceleration and deceleration during navigation.

- **PN Control**: The robot's linear and angular velocities are calculated as:
  \[
  \text{linear} = k_p \cdot d^{k_n}
  \]
  \[
  \text{angular} = k_p \cdot \theta^{k_n}
  \]
  Here, \( d \) is the distance to the target, \( \theta \) is the angle difference, and \( k_p \) and \( k_n \) are control parameters.

- **Rotation and Position Control**:
  - **Rotation**: The `rotateToHeading()` function adjusts the robot's heading to face the target.
  - **Position Control**: The `navigateToPosition()` function moves the robot to the target position while continuously checking for obstacles.

- **Obstacle Avoidance**: If the robot encounters an obstacle, it performs evasive maneuvers using the bumper sensors and recalculates its path.

#### End Condition Check
The robot uses an **End Condition Check** to determine when it has completed a full loop around the walls. It compares the total distance traveled to the minimum perimeter of the environment (19.48 meters) and ensures it returns to the starting position before transitioning to random navigation.

![Diagonal Corner Coordinates](https://github.com/user-attachments/assets/9e3d43de-cff7-4ad4-974d-73b31dc264c1)

![Get All Corners from Diagonal Coordinates](https://github.com/user-attachments/assets/fcbf61cd-738f-4852-83f8-1aaa319436f3)

#### Zig Zag (Original Idea)
The **Zig Zag** algorithm was initially designed to explore the inner area of the environment by following a zig-zag pattern. However, it was replaced with the **Biased Explore** algorithm due to edge cases such as getting stuck in dead ends or following the outer wall indefinitely.

- **Path Calculation**: The robot calculates a zig-zag path by dividing the longer sides of the environment into segments and moving between them.
- **Obstacle Avoidance**: The robot follows walls to avoid obstacles but often encountered issues in complex environments.

![Zigzag Pattern](https://github.com/user-attachments/assets/e74337bb-db70-4e4a-b62b-e69ab88ae2f4)

![Biased Exploration Target Point Selection](https://github.com/user-attachments/assets/9d283bf8-f524-4ebe-a4a9-e03cef61aa0d)

#### Biased Explore (Final Implementation)
The **Biased Explore** algorithm replaces the Zig Zag approach and is more efficient for exploring unknown environments. It uses the following steps:
1. **360° Scan**: The robot performs a full rotation to gather laser data.
2. **Target Scoring**: Each potential target is scored based on its distance from previously visited points.
3. **Navigation**: The robot moves to the highest-scoring target while avoiding obstacles.

---

## Future Recommendations

### 4.1 Algorithms - Frontier Exploration
Frontier-based exploration can enhance scanning accuracy in more complex environments. This approach is readily implementable with our current system, which continuously records and updates visited position coordinates. The team has already initiated sensor data acquisition, devising a vector to catalog all visited coordinates. Laser scan points are converted into an array of global coordinates by integrating the geometry of the current odometry with the laser angles. Subsequently, we calculate a target offset coordinate by adding an offset of 0.8 meters to the furthest laser scan data point, represented as the green point in Figure 4.1 (a) below.

![Frontier Exploration](https://github.com/user-attachments/assets/2b7e2445-bbd6-489c-a7e1-65aba5a0ba42)

For future enhancements, integrating an **occupancy grid** could refine the system's capability to discern the state of each coordinate or resolution grid, as shown in Figure 4.1 (b). This grid would classify visited coordinates as **"known space"** and coordinates beyond a certain threshold from the laser scans as **"unknown space."** The laser data delineating these areas would establish the **"frontier line."** Additionally, rather than using the furthest data point plus an offset as the next navigation target, calculating the nearest centroid of the frontier line could streamline the exploration process. This optimization would make the program more efficient by reducing unnecessary travel and improving path planning.

---

### 4.2 Hardware
The current hardware setup, while functional, has limitations that could be addressed to improve the robot's performance in future iterations:

- **Kinect 360 Sensor**: The Kinect sensor's narrow field of view (57°) limits the robot's ability to detect obstacles on its sides and localize accurately. This can lead to issues such as false rotations or jumps in the map during exploration.
  - **Recommendation**: Replace the Kinect sensor with a **360° LiDAR** (e.g., RPLidar A1M8). A LiDAR with a 360° field of view would provide more comprehensive data for localization and obstacle detection, significantly improving the robot's navigation capabilities.

- **Localization Issues**: The narrow field of view of the Kinect sensor often causes the gmapping algorithm to misinterpret the robot's position, leading to map rotations or overlaps during exploration.
  - **Recommendation**: A 360° LiDAR would provide more consistent and accurate data for localization, reducing the likelihood of map distortions.

- **Obstacle Interaction**: The robot struggles to navigate around obstacles or follow walls due to the limited field of view. For example, when a wall ends, the robot cannot see the side wall and must guess the turning radius, often leading to collisions.
  - **Recommendation**: A 360° LiDAR would allow the robot to detect side walls and follow them more accurately, reducing collisions and improving navigation efficiency.

- **Cost-Effectiveness**: Modern 360° LiDAR sensors, such as the RPLidar A1M8, are affordable (less than $100 CAD) and provide high accuracy, making them a cost-effective upgrade for future implementations.

---

By integrating frontier-based exploration algorithms and upgrading the hardware to include a 360° LiDAR, the robot's exploration and mapping capabilities can be significantly enhanced, making it more efficient and reliable in complex environments.

#### Movement (Final Implementation)
The final movement algorithm uses a **PN controller** to ensure smooth and efficient navigation:
- **Linear Movement**: The robot adjusts its speed based on the distance to the target.
- **Angular Movement**: The robot adjusts its heading to face the target while moving.
- **Obstacle Handling**: If an obstacle is detected, the robot slows down and recalculates its path.

---

## References
1. A. Grami, Chapter 20 - Finite-State Machines. in Discrete Mathematics. Academic Press, 2023.
2. T. Huang, “Slamtec - Sweeping Robot Path Planning Algorithm,” SLAMTEC.

---

### How to Run the Code
1. **Prerequisites**: Ensure you have ROS installed and the necessary packages (e.g., `kobuki_msgs`, `sensor_msgs`, `tf`).
2. **Clone the Repository**: Clone this repository to your ROS workspace.
3. **Build the Project**: Use `catkin_make` to build the project.
4. **Run the Node**: Launch the ROS node using `rosrun mie443_contest1 contest1`.
