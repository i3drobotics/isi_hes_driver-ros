cmake_minimum_required(VERSION 2.8.3)
project(isi_hes_driver)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf rospy isi_hes_msgs)

find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS
	CATKIN_DEPENDS roscpp std_msgs isi_hes_msgs
	LIBRARIES ${PROJECT_NAME}
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

add_dependencies(process_labview_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simulate_labview_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})