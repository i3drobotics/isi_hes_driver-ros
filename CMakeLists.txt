cmake_minimum_required(VERSION 2.8.3)
project(isi_hes_driver)

find_package(catkin REQUIRED roscpp std_msgs tf rospy)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
)

add_executable(
	process_labview_msgs src/process_labview_msgs.cpp
)
target_link_libraries(
	process_labview_msgs
	${catkin_LIBRARIES}
)

add_executable(
	simulate_labview_msgs src/simulate_labview_msgs.cpp
)
target_link_libraries(
	simulate_labview_msgs
	${catkin_LIBRARIES}
)

link_directories()
add_definitions(-std=c++11)
