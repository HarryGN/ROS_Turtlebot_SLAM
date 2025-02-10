cmake_minimum_required(VERSION 2.8)
project(mie443_contest1)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
<<<<<<< HEAD
add_executable(contest1 src/contest1.cpp src/coord.cpp src/wallFollow.cpp src/bumper_1.cpp src/common.cpp src/laser_1.cpp)
=======
add_executable(contest1 src/contest1.cpp src/coord.cpp src/common_1.cpp src/bumper_1.cpp src/laser_1.cpp src/wallFollow.cpp)
>>>>>>> 768b77a
target_link_libraries(contest1 ${catkin_LIBRARIES} ${OpenCV_LIB})

