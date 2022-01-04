cmake_minimum_required(VERSION 3.0.2)
project(gcode_rosutils)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp visualization_msgs
    DEPENDS EIGEN3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/plotting.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})