cmake_minimum_required(VERSION 3.14.4)
project(gcode_rviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(gcode_core REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions(-DQT_NO_KEYWORDS)

# gcode visualization widget
add_library(${PROJECT_NAME}_gcode_visualization_widget SHARED
  src/widget/gcode_visualization_widget.cpp
  src/widget/layer_range_visualization.cpp
  src/widget/layer_slider.cpp
  src/widget/color_list_editor.cpp

  include/gcode_rviz/widget/gcode_visualization_widget.h
  include/gcode_rviz/widget/layer_slider.h
  include/gcode_rviz/widget/color_list_editor.h
)

target_include_directories(${PROJECT_NAME}_gcode_visualization_widget PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_gcode_visualization_widget PUBLIC
  Qt5::Core
  Qt5::Widgets
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}_gcode_visualization_widget PUBLIC
  rclcpp
  pluginlib
  tf2_eigen
  tf2_ros
  pluginlib
  geometry_msgs
  visualization_msgs
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  gcode_core
)

# main RViz plugin library
add_library(${PROJECT_NAME} SHARED
  src/rviz/open_gcode_panel.cpp
  src/rviz/gcode_display.cpp

  include/gcode_rviz/rviz/open_gcode_panel.h
  include/gcode_rviz/rviz/gcode_display.h
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_gcode_visualization_widget
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  pluginlib
  tf2_eigen
  tf2_ros
  pluginlib
  geometry_msgs
  visualization_msgs
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  gcode_core
)

pluginlib_export_plugin_description_file(rviz_common gcode_rviz_plugin_description.xml)

# Install
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_gcode_visualization_widget
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES gcode_rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  pluginlib
  tf2_eigen
  tf2_ros
  geometry_msgs
  visualization_msgs
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  gcode_core
)

ament_package()
