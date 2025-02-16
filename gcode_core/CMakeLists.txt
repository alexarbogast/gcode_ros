cmake_minimum_required(VERSION 3.0.2)
project(gcode_core)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  gcode_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib gcode_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/core/gcode.cpp
    src/core/toolpath.cpp
    src/core/conversions.cpp
    src/core/interpreter.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

# build all examples, for now
add_executable(gcode_example docs/gcode_example.cpp)
target_link_libraries(gcode_example ${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(gcode_example PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(gcode_example SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})
