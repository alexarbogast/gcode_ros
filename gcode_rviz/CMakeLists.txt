cmake_minimum_required(VERSION 3.5.0)
project(gcode_rviz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  tf2_eigen
  tf2_ros
  geometry_msgs
  visualization_msgs
  eigen_conversions
  gcode_core
  pluginlib
)

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS
    include
    ${QT_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_gcode_visualization_widget
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rviz
    tf2_eigen
    tf2_ros
    pluginlib
    geometry_msgs
    visualization_msgs
    eigen_conversions
    gcode_core
  DEPENDS
    QT
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${QT_INCLUDE_DIR}
)

# gcode_visualization_widget
qt_wrap_cpp(${PROJECT_NAME}_gcode_visualization_widget_cpp_MOCS
  include/gcode_rviz/widget/gcode_visualization_widget.h
  include/gcode_rviz/widget/layer_slider.h
  include/gcode_rviz/widget/color_list_editor.h)

add_library(${PROJECT_NAME}_gcode_visualization_widget SHARED
  src/widget/gcode_visualization_widget.cpp
  src/widget/layer_range_visualization.cpp
  src/widget/layer_slider.cpp
  src/widget/color_list_editor.cpp
  ${${PROJECT_NAME}_gcode_visualization_widget_cpp_MOCS})

target_link_libraries(${PROJECT_NAME}_gcode_visualization_widget PUBLIC
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES})

target_include_directories(${PROJECT_NAME}_gcode_visualization_widget PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_gcode_visualization_widget SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR})

# open_gcode_panel
qt_wrap_cpp(${PROJECT_NAME}_cpp_MOCS
  include/gcode_rviz/rviz/open_gcode_panel.h
  include/gcode_rviz/rviz/gcode_display.h)

add_library(${PROJECT_NAME} SHARED
  src/rviz/open_gcode_panel.cpp
  src/rviz/gcode_display.cpp
  src/rviz/marker/toolpath.cpp
  src/rviz/ogre_helpers/line_list.cpp
  ${${PROJECT_NAME}_cpp_MOCS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_gcode_visualization_widget
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark plugin XML files for installation
install(FILES gcode_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
